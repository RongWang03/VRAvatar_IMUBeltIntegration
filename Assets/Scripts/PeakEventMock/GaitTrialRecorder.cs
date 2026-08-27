using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Records one walking trial at a time and scores it against a ground truth the wearer declares
/// BEFORE walking: a fixed number of steps, in strict alternation, starting on a stated foot.
///
/// Why the truth is declared rather than measured. Nothing on the belt can observe which foot
/// actually landed -- that is the thing under test, and scoring a method against its own output is
/// circular. Strict alternation from a known first foot makes the true leg a function of the step
/// INDEX alone, which the wearer controls and the sensors cannot influence.
///
/// Why two accuracy numbers. If a step is missed, every later verdict shifts by one index and
/// scores as wrong even when attribution is perfect. So strict accuracy is reported only for runs
/// where the detected count matches the target exactly, and alternation consistency -- the fraction
/// of consecutive verdicts that differ -- is reported for every run, because it survives a miscount
/// and degrades only when attribution itself is failing.
///
/// Observation-only. Nothing here drives the avatar or feeds back into detection.
/// </summary>
public sealed class GaitTrialRecorder : MonoBehaviour
{
    [Header("Wiring")]
    [SerializeField] private LegAttributionBySwingDominance attribution;

    [Header("Trial setup (set BEFORE starting a run)")]
    [Tooltip("Free-text condition label written into every row, e.g. slow60, normal85, fast110.")]
    [SerializeField] private string conditionLabel = "normal85";

    [Min(1)]
    [Tooltip("How many steps the wearer intends to take. Recall is measured against this.")]
    [SerializeField] private int targetSteps = 20;

    [Tooltip("The foot that takes step 1. Steps then alternate strictly.")]
    [SerializeField] private PeakLegSide startFoot = PeakLegSide.Right;

    [Tooltip("Flip the starting foot after each run, so a systematic left/right bias in the " +
             "method cannot hide behind always starting on the same side.")]
    [SerializeField] private bool autoAlternateStartFoot = true;

    [Header("Control")]
    [Tooltip("Press once to start a run, press again to stop and score it.")]
    [SerializeField] private KeyCode toggleKey = KeyCode.T;

    [Tooltip("OFF by default: the detector's Processing Gate owns the controller buttons. " +
             "Sharing one press between the two components made a single click start both, " +
             "with independent lockouts deciding whether that actually happened.")]
    [SerializeField] private bool useControllerInput = false;

    [Tooltip("Only used when Use Controller Input is on. Keep it clear of the detector's start " +
             "button, which is A on the right controller.")]
    [SerializeField] private OVRInput.RawButton toggleButton = OVRInput.RawButton.B;

    [SerializeField] private OVRInput.Controller toggleController = OVRInput.Controller.RTouch;

    [Min(0.2f)]
    [Tooltip("Seconds of lockout after a press, so one physical press cannot start and stop a run.")]
    [SerializeField] private float pressLockout = 0.6f;

    [Header("Output")]
    [SerializeField] private bool writeCsv = true;
    [SerializeField] private string folderName = "PeakEventLogs/Trials";
    [SerializeField] private bool logToConsole = true;

    [Header("Live (read-only)")]
    [SerializeField] private bool running;
    [SerializeField] private int stepsDetected;
    [SerializeField] private string lastRow = "";

    [Header("Last result (read-only)")]
    [SerializeField] private string lastSummary = "";
    [SerializeField] private float recallPercent;
    [SerializeField] private int extraSteps;
    [SerializeField] private bool strictScoringValid;
    [SerializeField] private float strictAccuracyPercent;
    [SerializeField] private float alternationConsistencyPercent;
    [SerializeField] private float confidentCoveragePercent;
    [SerializeField] private float confidentAccuracyPercent;
    [SerializeField] private float meanIntervalSeconds;
    [SerializeField] private float impliedCadenceSpm;

    private struct Row
    {
        public double Time;
        public PeakLegSide Verdict;
        public PeakLegSide Truth;
        public float Ratio;
        public bool Confident;
        public float Peak0;
        public float Peak1;
    }

    private readonly List<Row> rows = new List<Row>();
    private double runStartTime;
    private double lastPressTime = -10.0;
    private int runIndex;
    private bool subscribed;
    private string runStamp = "";

    private void Awake()
    {
        if (attribution == null)
        {
            attribution = FindFirstObjectByType<LegAttributionBySwingDominance>();
        }
    }

    private void OnEnable()
    {
        if (attribution == null)
        {
            Debug.LogWarning(
                "[GaitTrialRecorder] No LegAttributionBySwingDominance found; inert.",
                this);

            return;
        }

        attribution.VerdictProduced += HandleVerdict;
        subscribed = true;
    }

    private void OnDisable()
    {
        if (subscribed)
        {
            attribution.VerdictProduced -= HandleVerdict;
            subscribed = false;
        }

        // A run left open by exiting Play is still real data, so it is scored and written rather
        // than discarded.
        if (running)
        {
            StopRun();
        }
    }

    private void Update()
    {
        double now = Time.realtimeSinceStartupAsDouble;

        if (now - lastPressTime < pressLockout)
        {
            return;
        }

        bool pressed = Input.GetKeyDown(toggleKey);

        if (!pressed && useControllerInput)
        {
            try
            {
                pressed = OVRInput.GetDown(toggleButton, toggleController);
            }
            catch (Exception)
            {
                // No OVR runtime in this context; the keyboard path still works.
            }
        }

        if (!pressed)
        {
            return;
        }

        lastPressTime = now;

        if (running)
        {
            StopRun();
        }
        else
        {
            StartRun();
        }
    }

    [ContextMenu("Start Run")]
    public void StartRun()
    {
        rows.Clear();
        stepsDetected = 0;
        lastRow = "";
        runStartTime = Time.realtimeSinceStartupAsDouble;
        runStamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        runIndex++;
        running = true;

        if (logToConsole)
        {
            Debug.Log(
                $"[GaitTrial] RUN {runIndex} START | condition={conditionLabel} " +
                $"target={targetSteps} startFoot={startFoot} -- walk now, count aloud.",
                this);
        }
    }

    [ContextMenu("Stop Run")]
    public void StopRun()
    {
        if (!running)
        {
            return;
        }

        running = false;
        Score();
        WriteRunCsv();
        AppendSummaryCsv();

        if (logToConsole)
        {
            Debug.Log($"[GaitTrial] RUN {runIndex} END | {lastSummary}", this);
        }

        if (autoAlternateStartFoot)
        {
            startFoot = startFoot == PeakLegSide.Left ? PeakLegSide.Right : PeakLegSide.Left;
        }
    }

    private void HandleVerdict(LegAttributionBySwingDominance.LegVerdict verdict)
    {
        if (!running)
        {
            return;
        }

        stepsDetected++;

        Row row = new Row
        {
            Time = verdict.UnityTimestamp,
            Verdict = verdict.Leg,
            Truth = TruthForStep(stepsDetected),
            Ratio = verdict.Ratio,
            Confident = verdict.Confident,
            Peak0 = verdict.PeakBoard0,
            Peak1 = verdict.PeakBoard1,
        };

        rows.Add(row);

        lastRow =
            $"#{stepsDetected} verdict={row.Verdict} truth={row.Truth} " +
            $"{(row.Verdict == row.Truth ? "OK" : "WRONG")} ratio={row.Ratio:F2} " +
            $"{(row.Confident ? "CONFIDENT" : "AMBIGUOUS")}";

        if (logToConsole)
        {
            Debug.Log($"[GaitTrial] {lastRow}", this);
        }
    }

    // Strict alternation: step 1 is the declared foot, and every step after it swaps.
    private PeakLegSide TruthForStep(int oneBasedIndex)
    {
        bool onStartFoot = (oneBasedIndex % 2) == 1;

        if (onStartFoot)
        {
            return startFoot;
        }

        return startFoot == PeakLegSide.Left ? PeakLegSide.Right : PeakLegSide.Left;
    }

    private void Score()
    {
        int n = rows.Count;

        recallPercent = targetSteps > 0
            ? 100f * Mathf.Min(n, targetSteps) / targetSteps
            : 0f;

        extraSteps = Mathf.Max(0, n - targetSteps);

        // Index-aligned truth is only meaningful when nothing was missed or added. Reporting a
        // strict score on a miscounted run would blame attribution for a detection failure.
        strictScoringValid = n == targetSteps;

        int correct = 0;
        int confident = 0;
        int confidentCorrect = 0;

        for (int i = 0; i < n; i++)
        {
            bool hit = rows[i].Verdict == rows[i].Truth;

            if (hit)
            {
                correct++;
            }

            if (rows[i].Confident)
            {
                confident++;

                if (hit)
                {
                    confidentCorrect++;
                }
            }
        }

        strictAccuracyPercent = strictScoringValid && n > 0 ? 100f * correct / n : 0f;
        confidentCoveragePercent = n > 0 ? 100f * confident / n : 0f;

        confidentAccuracyPercent = strictScoringValid && confident > 0
            ? 100f * confidentCorrect / confident
            : 0f;

        int pairs = 0;
        int alternating = 0;

        for (int i = 1; i < n; i++)
        {
            pairs++;

            if (rows[i].Verdict != rows[i - 1].Verdict)
            {
                alternating++;
            }
        }

        alternationConsistencyPercent = pairs > 0 ? 100f * alternating / pairs : 0f;

        double intervalSum = 0.0;
        int intervalCount = 0;

        for (int i = 1; i < n; i++)
        {
            double gap = rows[i].Time - rows[i - 1].Time;

            if (gap > 0.15 && gap < 3.0)
            {
                intervalSum += gap;
                intervalCount++;
            }
        }

        meanIntervalSeconds = intervalCount > 0 ? (float)(intervalSum / intervalCount) : 0f;
        impliedCadenceSpm = meanIntervalSeconds > 0.01f ? 60f / meanIntervalSeconds : 0f;

        string strictText = strictScoringValid
            ? $"{strictAccuracyPercent:F0}%"
            : "n/a (count mismatch)";

        string confidentText = strictScoringValid && confident > 0
            ? $", {confidentAccuracyPercent:F0}% correct"
            : "";

        lastSummary =
            $"condition={conditionLabel} start={startFoot} " +
            $"detected={n}/{targetSteps} recall={recallPercent:F0}% extra={extraSteps} " +
            $"| strict={strictText} alternation={alternationConsistencyPercent:F0}% " +
            $"| confident {confidentCoveragePercent:F0}% coverage{confidentText} " +
            $"| cadence={impliedCadenceSpm:F0} spm";
    }

    private string ResolveFolder()
    {
        string folder = Path.Combine(Application.dataPath, "..", folderName);
        Directory.CreateDirectory(folder);
        return folder;
    }

    private void WriteRunCsv()
    {
        if (!writeCsv || rows.Count == 0)
        {
            return;
        }

        try
        {
            CultureInfo inv = CultureInfo.InvariantCulture;
            string path = Path.Combine(ResolveFolder(), $"Trial_{conditionLabel}_{runStamp}.csv");

            StringBuilder sb = new StringBuilder();
            sb.AppendLine(
                "step_index,time_since_start_s,interval_s,verdict,truth,correct,ratio," +
                "confident,peak_board0,peak_board1");

            for (int i = 0; i < rows.Count; i++)
            {
                double sinceStart = rows[i].Time - runStartTime;
                double interval = i > 0 ? rows[i].Time - rows[i - 1].Time : double.NaN;

                sb.AppendLine(
                    (i + 1).ToString(inv) + "," +
                    sinceStart.ToString("F4", inv) + "," +
                    (double.IsNaN(interval) ? "" : interval.ToString("F4", inv)) + "," +
                    rows[i].Verdict + "," +
                    rows[i].Truth + "," +
                    (rows[i].Verdict == rows[i].Truth ? "1" : "0") + "," +
                    rows[i].Ratio.ToString("F3", inv) + "," +
                    (rows[i].Confident ? "1" : "0") + "," +
                    rows[i].Peak0.ToString("F3", inv) + "," +
                    rows[i].Peak1.ToString("F3", inv));
            }

            File.WriteAllText(path, sb.ToString(), Encoding.UTF8);

            if (logToConsole)
            {
                Debug.Log($"[GaitTrial] wrote {path}", this);
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[GaitTrialRecorder] Could not write run CSV: {e.Message}", this);
        }
    }

    // One row per run, appended across the whole session, so the nine runs land in a single file
    // ready to read as a table with no hand transcription.
    private void AppendSummaryCsv()
    {
        if (!writeCsv)
        {
            return;
        }

        try
        {
            CultureInfo inv = CultureInfo.InvariantCulture;
            string path = Path.Combine(ResolveFolder(), "TrialSummary.csv");

            if (!File.Exists(path))
            {
                File.WriteAllText(
                    path,
                    "timestamp,condition,run_index,start_foot,target_steps,detected_steps," +
                    "recall_percent,extra_steps,strict_valid,strict_accuracy_percent," +
                    "alternation_consistency_percent,confident_coverage_percent," +
                    "confident_accuracy_percent,mean_interval_s,implied_cadence_spm\n",
                    Encoding.UTF8);
            }

            string line =
                DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + "," +
                conditionLabel + "," +
                runIndex.ToString(inv) + "," +
                startFoot + "," +
                targetSteps.ToString(inv) + "," +
                rows.Count.ToString(inv) + "," +
                recallPercent.ToString("F1", inv) + "," +
                extraSteps.ToString(inv) + "," +
                (strictScoringValid ? "1" : "0") + "," +
                (strictScoringValid ? strictAccuracyPercent.ToString("F1", inv) : "") + "," +
                alternationConsistencyPercent.ToString("F1", inv) + "," +
                confidentCoveragePercent.ToString("F1", inv) + "," +
                (strictScoringValid ? confidentAccuracyPercent.ToString("F1", inv) : "") + "," +
                meanIntervalSeconds.ToString("F4", inv) + "," +
                impliedCadenceSpm.ToString("F1", inv) + "\n";

            File.AppendAllText(path, line, Encoding.UTF8);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[GaitTrialRecorder] Could not append summary CSV: {e.Message}", this);
        }
    }
}
