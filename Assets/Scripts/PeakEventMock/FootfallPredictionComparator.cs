using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Measures how far the gait controller's PREDICTED foot landings sit from the landings the belt
/// actually felt.
///
/// This is the one comparison the hip-mounted hardware supports honestly. Leg attribution cannot
/// be measured from these sensors -- both boards feel both feet, because the heel-strike shock
/// crosses the pelvis -- but the TIMING of a footfall is a real, independent measurement, and the
/// controller has been predicting landing times all along with nothing checking them.
///
/// Pairing is by time alone, nearest-neighbour within a window. Deliberately not by leg: the
/// measured side has no leg, and borrowing one from the controller's own state would make the
/// comparison circular.
///
/// Observation-only. Nothing here influences the avatar.
/// </summary>
public sealed class FootfallPredictionComparator : MonoBehaviour
{
    [Header("Wiring")]
    [SerializeField] private ImuGaitEventDetector detector;

    [SerializeField]
    private PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth trajectoryController;

    [Header("Pairing")]
    [Min(0.05f)]
    [Tooltip("Seconds. A measured footfall pairs with the nearest predicted landing inside this " +
             "window. Must stay well below one footfall interval (~0.5 s), or a late prediction " +
             "could steal the NEXT footfall's partner and quietly halve the reported error.")]
    [SerializeField] private float pairingWindow = 0.25f;

    [Header("Output")]
    [SerializeField] private bool writeCsv = true;
    [SerializeField] private string folderName = "PeakEventLogs/Proposed";
    [SerializeField] private bool logToConsole = true;

    [Header("Results (read-only)")]
    [SerializeField] private int measuredFootfalls;
    [SerializeField] private int predictedLandings;
    [SerializeField] private int pairedCount;
    [SerializeField] private int unpairedMeasured;
    [SerializeField] private int unpairedPredicted;

    [Tooltip("Milliseconds. Positive means the prediction came LATE relative to the real footfall.")]
    [SerializeField] private float meanErrorMs;

    [SerializeField] private float meanAbsErrorMs;
    [SerializeField] private float stdDevMs;
    [SerializeField] private float worstAbsErrorMs;

    private readonly List<double> pendingPredictions = new List<double>();
    private readonly List<PeakLegSide> pendingPredictionLegs = new List<PeakLegSide>();
    private readonly List<double> pendingMeasurements = new List<double>();
    private readonly List<float> pendingMeasurementShocks = new List<float>();
    private readonly List<float> errorsMs = new List<float>();

    private StreamWriter writer;
    private string filePath;
    private bool subscribed;

    private void Awake()
    {
        if (detector == null)
        {
            detector = FindFirstObjectByType<ImuGaitEventDetector>();
        }

        if (trajectoryController == null)
        {
            trajectoryController =
                FindFirstObjectByType<
                    PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth>();
        }
    }

    private void OnEnable()
    {
        if (detector == null || trajectoryController == null)
        {
            Debug.LogWarning(
                "[FootfallPredictionComparator] Detector or trajectory controller missing; inert.",
                this);

            return;
        }

        detector.FootfallMeasured += HandleMeasuredFootfall;
        trajectoryController.FootLanded += HandlePredictedLanding;
        subscribed = true;

        if (writeCsv)
        {
            OpenCsv();
        }
    }

    private void OnDisable()
    {
        if (subscribed)
        {
            detector.FootfallMeasured -= HandleMeasuredFootfall;
            trajectoryController.FootLanded -= HandlePredictedLanding;
            subscribed = false;
        }

        CloseCsv();
    }

    private void HandlePredictedLanding(
        PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth.FootLandingEvent landing)
    {
        predictedLandings++;

        pendingPredictions.Add(landing.UnityTimestamp);
        pendingPredictionLegs.Add(landing.Leg);

        TryPair();
        ExpireStale(landing.UnityTimestamp);
    }

    private void HandleMeasuredFootfall(ImuGaitEventDetector.MeasuredFootfall footfall)
    {
        measuredFootfalls++;

        pendingMeasurements.Add(footfall.UnityTimestamp);
        pendingMeasurementShocks.Add(footfall.Shock);

        TryPair();
        ExpireStale(footfall.UnityTimestamp);
    }

    // Both sides are buffered, because either can arrive first: a prediction is raised when the
    // controller's quarter elapses, a measurement when the shock crosses threshold, and which
    // comes first depends on whether the avatar is ahead of or behind the wearer. Matching only
    // forwards would silently discard every case where the real footfall led the prediction --
    // which is exactly the case worth measuring.
    private void TryPair()
    {
        while (true)
        {
            int bestP = -1;
            int bestM = -1;
            double bestAbs = double.MaxValue;

            for (int pi = 0; pi < pendingPredictions.Count; pi++)
            {
                for (int mi = 0; mi < pendingMeasurements.Count; mi++)
                {
                    double abs = Math.Abs(pendingPredictions[pi] - pendingMeasurements[mi]);

                    if (abs < bestAbs && abs <= pairingWindow)
                    {
                        bestAbs = abs;
                        bestP = pi;
                        bestM = mi;
                    }
                }
            }

            if (bestP < 0)
            {
                return;
            }

            double predicted = pendingPredictions[bestP];
            double measured = pendingMeasurements[bestM];
            PeakLegSide leg = pendingPredictionLegs[bestP];
            float shock = pendingMeasurementShocks[bestM];

            pendingPredictions.RemoveAt(bestP);
            pendingPredictionLegs.RemoveAt(bestP);
            pendingMeasurements.RemoveAt(bestM);
            pendingMeasurementShocks.RemoveAt(bestM);

            double errorSeconds = predicted - measured;

            pairedCount++;
            errorsMs.Add((float)(errorSeconds * 1000.0));
            RecomputeStatistics();

            WriteRow(measured, predicted, leg.ToString(), shock, "paired");

            if (logToConsole)
            {
                Debug.Log(
                    "[FootfallPredictionComparator] " +
                    $"measured={measured:F3} predicted={predicted:F3} " +
                    $"error={errorSeconds * 1000.0:F1} ms (predicted leg={leg}) " +
                    $"| n={pairedCount} meanAbs={meanAbsErrorMs:F1} ms sd={stdDevMs:F1} ms",
                    this);
            }
        }
    }

    // Only declared unpaired once no future partner could still fall inside the window.
    private void ExpireStale(double now)
    {
        for (int i = pendingPredictions.Count - 1; i >= 0; i--)
        {
            if (now - pendingPredictions[i] > pairingWindow)
            {
                pendingPredictions.RemoveAt(i);
                pendingPredictionLegs.RemoveAt(i);
                unpairedPredicted++;
            }
        }

        for (int i = pendingMeasurements.Count - 1; i >= 0; i--)
        {
            if (now - pendingMeasurements[i] > pairingWindow)
            {
                WriteRow(
                    pendingMeasurements[i],
                    double.NaN,
                    "",
                    pendingMeasurementShocks[i],
                    "unpaired_measured");

                pendingMeasurements.RemoveAt(i);
                pendingMeasurementShocks.RemoveAt(i);
                unpairedMeasured++;
            }
        }
    }

    private void RecomputeStatistics()
    {
        if (errorsMs.Count == 0)
        {
            return;
        }

        float sum = 0f;
        float sumAbs = 0f;
        float worst = 0f;

        foreach (float error in errorsMs)
        {
            sum += error;
            sumAbs += Mathf.Abs(error);

            if (Mathf.Abs(error) > worst)
            {
                worst = Mathf.Abs(error);
            }
        }

        meanErrorMs = sum / errorsMs.Count;
        meanAbsErrorMs = sumAbs / errorsMs.Count;
        worstAbsErrorMs = worst;

        float variance = 0f;

        foreach (float error in errorsMs)
        {
            float d = error - meanErrorMs;
            variance += d * d;
        }

        stdDevMs = Mathf.Sqrt(variance / errorsMs.Count);
    }

    private void OpenCsv()
    {
        try
        {
            string folder = Path.Combine(Application.dataPath, "..", folderName);
            Directory.CreateDirectory(folder);

            filePath = Path.Combine(
                folder,
                $"FootfallPredictionError_{DateTime.Now:yyyyMMdd_HHmmss}.csv");

            writer = new StreamWriter(filePath, false, Encoding.UTF8);
            writer.WriteLine(
                "measured_unity_time,predicted_unity_time,error_ms,predicted_leg,shock,status");
            writer.Flush();

            Debug.Log($"[FootfallPredictionComparator] Recording to {filePath}", this);
        }
        catch (Exception e)
        {
            Debug.LogWarning(
                $"[FootfallPredictionComparator] Could not open CSV: {e.Message}",
                this);

            writer = null;
        }
    }

    private void WriteRow(
        double measured,
        double predicted,
        string leg,
        float shock,
        string status)
    {
        if (writer == null)
        {
            return;
        }

        CultureInfo inv = CultureInfo.InvariantCulture;

        string predictedText = double.IsNaN(predicted) ? "" : predicted.ToString("F4", inv);

        string errorText = double.IsNaN(predicted)
            ? ""
            : ((predicted - measured) * 1000.0).ToString("F2", inv);

        writer.WriteLine(
            measured.ToString("F4", inv) + "," +
            predictedText + "," +
            errorText + "," +
            leg + "," +
            shock.ToString("F3", inv) + "," +
            status);

        writer.Flush();
    }

    private void CloseCsv()
    {
        if (writer == null)
        {
            return;
        }

        writer.Flush();
        writer.Dispose();
        writer = null;

        Debug.Log(
            $"[FootfallPredictionComparator] Stopped. paired={pairedCount} " +
            $"meanAbs={meanAbsErrorMs:F1} ms sd={stdDevMs:F1} ms worst={worstAbsErrorMs:F1} ms " +
            $"-> {filePath}",
            this);
    }
}
