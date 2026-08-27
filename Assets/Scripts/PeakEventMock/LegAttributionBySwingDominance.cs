using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Decides which leg landed by asking which thigh was MOVING MORE in the moment before the
/// footfall, rather than by detecting a peak on either leg.
///
/// Why comparison and not a threshold. The absolute swing rate on this hardware ranges from about
/// 0.8 to 3.3 rad/s depending on pace, mounting and wearer, while the crosstalk picked up by the
/// planted leg sits around 0.75. A threshold has to land inside that moving window, is derived
/// from one walk and applied to another, and was measured landing on the wrong side of it in both
/// directions. The RATIO between swinging and planted leg stayed near 3:1 throughout, because
/// both terms scale together -- so the ratio is the stable quantity and the absolute is not.
///
/// Consequences of that choice: no threshold, no calibration dependency, no zero crossing, no
/// refractory, no drift correction. The raw gyro magnitude is used directly, so even the swing
/// axis and its sign are irrelevant here.
///
/// The comparison is on RAW peaks. Normalising each board by its own running average was tried,
/// to cancel the roughly 2:1 difference in what each mounting transmits, and measured worse:
/// 19/19 correct raw against 16/19 normalised over single-leg trials. The running average rises
/// for whichever leg has been moving, so it discounts exactly the evidence under test. The toggle
/// remains for a badly mismatched pair of mountings, but defaults off.
///
/// Observation-only. Nothing here drives the avatar.
/// </summary>
public sealed class LegAttributionBySwingDominance : MonoBehaviour
{
    /// <summary>
    /// One attribution verdict, published so recorders can score it against a known ground truth
    /// without this class needing to know anything about trials.
    /// </summary>
    public readonly struct LegVerdict
    {
        public LegVerdict(
            double unityTimestamp,
            PeakLegSide leg,
            float ratio,
            bool confident,
            float peakBoard0,
            float peakBoard1)
        {
            UnityTimestamp = unityTimestamp;
            Leg = leg;
            Ratio = ratio;
            Confident = confident;
            PeakBoard0 = peakBoard0;
            PeakBoard1 = peakBoard1;
        }

        public double UnityTimestamp { get; }
        public PeakLegSide Leg { get; }
        public float Ratio { get; }
        public bool Confident { get; }
        public float PeakBoard0 { get; }
        public float PeakBoard1 { get; }
    }

    /// <summary>Raised once per measured footfall, whether or not the verdict is confident.</summary>
    public event System.Action<LegVerdict> VerdictProduced;

    [Header("Wiring")]
    [SerializeField] private ImuGaitEventDetector detector;
    [SerializeField] private imu_data_receive imuSource;

    [Header("Comparison")]
    [Min(0.05f)]
    [Tooltip("Seconds of history compared at each footfall. Should cover the swing that just " +
             "ended -- roughly a quarter of a gait cycle -- without reaching back into the " +
             "previous step, when the other leg was the one swinging.")]
    [SerializeField] private float windowSeconds = 0.45f;

    [Min(1.0f)]
    [Tooltip("How much louder the winning board must be before the verdict counts as confident. " +
             "Measured swinging-to-planted ratio on this hardware was 1.46 to 4.4 over 19 " +
             "single-leg trials, mean about 3.2, so 1.4 accepts every correct call while still " +
             "refusing genuinely close ones.")]
    [SerializeField] private float minRatio = 1.4f;

    [Tooltip("Divide each board's window peak by its own running average before comparing. " +
             "OFF by default on measured evidence: single-leg tests scored 19/19 on raw peaks " +
             "and 16/19 with normalisation on, because the running average rises for whichever " +
             "leg has been moving and so penalises the very leg it should favour. Only worth " +
             "enabling if the two mountings become badly mismatched.")]
    [SerializeField] private bool normaliseByBoardScale = false;

    [Min(1f)]
    [Tooltip("Seconds. Must be far longer than a stride, or the normaliser tracks gait instead " +
             "of mounting. Only used when Normalise By Board Scale is on.")]
    [SerializeField] private float normaliseTau = 60f;

    [Header("Output")]
    [SerializeField] private bool logVerdicts = true;

    [Tooltip("Push each verdict to the detector so the contact beep is pitched by THIS method " +
             "rather than by the older angle and timing ones, which measured ~20 degrees of " +
             "separation where 45 was needed and about 45% correct respectively.")]
    [SerializeField] private bool driveBeepPitch = true;

    [Header("Results (read-only)")]
    [SerializeField] private string lastVerdict = "";
    [SerializeField] private float lastRatio;
    [SerializeField] private int confidentCount;
    [SerializeField] private int ambiguousCount;
    [SerializeField] private int leftVerdicts;
    [SerializeField] private int rightVerdicts;

    [Tooltip("Peak |gyro| seen inside the window at the last footfall, per board.")]
    [SerializeField] private float[] lastWindowPeak;

    [Tooltip("The values actually compared. Equal to the window peaks unless normalisation is on.")]
    [SerializeField] private float[] lastNormalised;

    [Tooltip("Running average |gyro| per board. Reflects mounting, not gait.")]
    [SerializeField] private float[] boardScale;

    private struct Sample
    {
        public double Time;
        public float Magnitude;
    }

    private List<Sample>[] history;
    private bool[] hasScale;
    private int boardCount;
    private bool subscribed;

    private void Awake()
    {
        if (detector == null)
        {
            detector = FindFirstObjectByType<ImuGaitEventDetector>();
        }

        if (imuSource == null)
        {
            imuSource = FindFirstObjectByType<imu_data_receive>();
        }
    }

    private void OnEnable()
    {
        if (detector == null || imuSource == null)
        {
            Debug.LogWarning(
                "[LegAttributionBySwingDominance] Detector or IMU source missing; inert.",
                this);

            return;
        }

        imuSource.SampleFused += HandleSample;
        detector.FootfallMeasured += HandleFootfall;
        subscribed = true;
    }

    private void OnDisable()
    {
        if (!subscribed)
        {
            return;
        }

        imuSource.SampleFused -= HandleSample;
        detector.FootfallMeasured -= HandleFootfall;
        subscribed = false;
    }

    // Raw gyro magnitude, deliberately. Not the calibrated swing-axis projection: magnitude needs
    // no axis, no sign and no bias correction, so this path cannot inherit a calibration error.
    private void HandleSample(imu_data_receive.ImuSampleSnapshot sample)
    {
        EnsureBoardState(sample.Board + 1);

        int board = sample.Board;
        float magnitude = sample.AngularVelocity.magnitude;
        double now = Time.realtimeSinceStartupAsDouble;

        history[board].Add(new Sample { Time = now, Magnitude = magnitude });

        double cutoff = now - windowSeconds;
        int drop = 0;

        while (drop < history[board].Count && history[board][drop].Time < cutoff)
        {
            drop++;
        }

        if (drop > 0)
        {
            history[board].RemoveRange(0, drop);
        }

        float dt = sample.HasDeltaTime ? sample.DeltaTime : 0.01f;
        float alpha = 1f - Mathf.Exp(-dt / Mathf.Max(1f, normaliseTau));

        if (!hasScale[board])
        {
            boardScale[board] = magnitude;
            hasScale[board] = true;
        }
        else
        {
            boardScale[board] += alpha * (magnitude - boardScale[board]);
        }
    }

    private void HandleFootfall(ImuGaitEventDetector.MeasuredFootfall footfall)
    {
        if (boardCount < 2)
        {
            return;
        }

        int winner = -1;
        float best = 0f;
        float second = 0f;

        for (int board = 0; board < boardCount; board++)
        {
            float peak = WindowPeak(board);
            lastWindowPeak[board] = peak;

            // Raw peak by default. Dividing by the board's own running level was intended to
            // cancel a mounting mismatch, but measured worse: the running average rises for the
            // leg that has been moving, so it discounts exactly the evidence being tested.
            float normalised = normaliseByBoardScale
                ? peak / Mathf.Max(0.01f, boardScale[board])
                : peak;

            lastNormalised[board] = normalised;

            if (normalised > best)
            {
                second = best;
                best = normalised;
                winner = board;
            }
            else if (normalised > second)
            {
                second = normalised;
            }
        }

        if (winner < 0)
        {
            return;
        }

        float ratio = second > 0.0001f ? best / second : float.PositiveInfinity;
        lastRatio = ratio;

        PeakLegSide leg = detector.ResolveLegForBoard(winner);
        lastVerdict = leg.ToString();

        bool confident = ratio >= minRatio;

        if (confident)
        {
            confidentCount++;
        }
        else
        {
            ambiguousCount++;
        }

        if (leg == PeakLegSide.Left)
        {
            leftVerdicts++;
        }
        else
        {
            rightVerdicts++;
        }

        // Pushed before the event, so the detector -- which raised FootfallMeasured a few lines
        // earlier and plays the beep a few lines later -- has this verdict in hand for the same
        // footfall rather than the previous one.
        if (driveBeepPitch)
        {
            detector.ReportExternalLegVerdict(leg, confident);
        }

        VerdictProduced?.Invoke(
            new LegVerdict(
                footfall.UnityTimestamp,
                leg,
                ratio,
                confident,
                lastWindowPeak.Length > 0 ? lastWindowPeak[0] : 0f,
                lastWindowPeak.Length > 1 ? lastWindowPeak[1] : 0f));

        if (logVerdicts)
        {
            Debug.Log(
                $"[SwingDominance] t={footfall.UnityTimestamp:F3} leg={leg} " +
                $"| peaks {lastWindowPeak[0]:F2} vs {lastWindowPeak[1]:F2} rad/s " +
                $"| normalised {lastNormalised[0]:F2} vs {lastNormalised[1]:F2} " +
                $"| ratio={ratio:F2} {(confident ? "CONFIDENT" : "AMBIGUOUS")} " +
                $"| L={leftVerdicts} R={rightVerdicts}",
                this);
        }
    }

    private float WindowPeak(int board)
    {
        float peak = 0f;

        List<Sample> samples = history[board];

        for (int i = 0; i < samples.Count; i++)
        {
            if (samples[i].Magnitude > peak)
            {
                peak = samples[i].Magnitude;
            }
        }

        return peak;
    }

    private void EnsureBoardState(int required)
    {
        if (required <= boardCount && history != null)
        {
            return;
        }

        int count = Mathf.Max(required, boardCount);

        List<Sample>[] newHistory = new List<Sample>[count];
        bool[] newHasScale = new bool[count];
        float[] newScale = new float[count];
        float[] newPeak = new float[count];
        float[] newNormalised = new float[count];

        for (int i = 0; i < count; i++)
        {
            newHistory[i] = i < boardCount && history != null
                ? history[i]
                : new List<Sample>();

            if (i < boardCount)
            {
                newHasScale[i] = hasScale[i];
                newScale[i] = boardScale[i];
                newPeak[i] = lastWindowPeak[i];
                newNormalised[i] = lastNormalised[i];
            }
        }

        history = newHistory;
        hasScale = newHasScale;
        boardScale = newScale;
        lastWindowPeak = newPeak;
        lastNormalised = newNormalised;
        boardCount = count;
    }
}
