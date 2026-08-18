using System.Collections;
using UnityEngine;

/// <summary>
/// Turns the continuous hip-pocket IMU stream into the discrete leg events the gait pipeline
/// understands, so the avatar walks without anyone pressing a controller button.
///
/// Signal choice: detection runs on the RAW GYRO, not on the fused quaternion. The complementary
/// filter in imu_data_receive only accepts the accelerometer as a gravity reference while
/// |accel| is within a few percent of 9.81, and during walking that gate rejects almost every
/// sample -- so the fused orientation degrades to pure gyro integration and its absolute pitch
/// drifts. Fine for a debug cube, unusable as a threshold input.
///
/// "Top" (peak hip flexion, thigh at its highest) is the turning point of the swing, which on the
/// medio-lateral gyro axis is a sign change from flexion-positive to extension-negative. Zero
/// crossings are immune to the drift that would slowly poison an angle threshold.
///
/// This component never touches an Avatar, Foot Target, bone, IK target, or gait controller
/// state. Its only output is JasmineInputAdapter.SubmitSemanticEvent.
/// </summary>
public sealed class ImuGaitEventDetector : MonoBehaviour
{
    public enum GyroAxis
    {
        X,
        Y,
        Z
    }

    [Header("Wiring")]
    [SerializeField] private imu_data_receive imuSource;
    [SerializeField] private JasmineInputAdapter jasmineAdapter;

    [Header("Board Mapping")]
    [Tooltip("Which leg each board observes. As worn: board 0 = RIGHT hip pocket, " +
             "board 1 = LEFT hip pocket. Note this is the reverse of the obvious guess.")]
    [SerializeField]
    private PeakLegSide[] boardToLeg =
    {
        PeakLegSide.Right,
        PeakLegSide.Left
    };

    [Tooltip("Which gyro axis is medio-lateral for each board. Depends on how the board sits in " +
             "the pocket, so it varies per wear -- run Calibrate Detector rather than guessing.")]
    [SerializeField]
    private GyroAxis[] boardGyroAxis =
    {
        GyroAxis.X,
        GyroAxis.X
    };

    [Tooltip("Flip when the chosen axis reads flexion as negative. Calibration sets this.")]
    [SerializeField]
    private bool[] boardInvert =
    {
        false,
        false
    };

    [Header("Detection")]
    [Tooltip("Master switch. When false, samples are still analysed for diagnostics but no " +
             "event is ever submitted.")]
    [SerializeField] private bool enableDetector = false;

    [Min(0f)]
    [Tooltip("rad/s. The swing must peak above this before its turning point counts as a step. " +
             "Rejects standing sway, shuffling, and fidgeting.")]
    [SerializeField] private float minSwingRate = 1.2f;

    [Min(0f)]
    [Tooltip("Seconds. Per-leg refractory period; suppresses double-fires on a noisy stride. " +
             "Must stay below the fastest same-leg cycle the controller accepts (0.7 s).")]
    [SerializeField] private float minEventInterval = 0.35f;

    [Tooltip("Reject a Top for the same leg twice in a row. The gait controller enforces strict " +
             "alternation anyway, but rejecting here makes a double-fire visible as a detector " +
             "counter instead of an unexplained stutter.")]
    [SerializeField] private bool enforceAlternation = true;

    [Header("Optional Event Types")]
    [Tooltip("HitBottom is observation-only downstream and roughly doubles the log volume.")]
    [SerializeField] private bool emitHitBottom = false;

    [Tooltip("StartGoingUp is observation-only downstream.")]
    [SerializeField] private bool emitStartGoingUp = false;

    [Header("Calibration")]
    [Min(0.5f)]
    [Tooltip("Seconds of standing still used to measure gyro bias.")]
    [SerializeField] private float stillCalibrationSeconds = 3f;

    [Min(1f)]
    [Tooltip("Seconds of walking used to pick the swing axis and set the threshold.")]
    [SerializeField] private float walkCalibrationSeconds = 5f;

    [Range(0.1f, 0.9f)]
    [Tooltip("minSwingRate is set to this fraction of the peak rate seen while walking.")]
    [SerializeField] private float thresholdFraction = 0.35f;

    [Header("Diagnostics (read-only)")]
    [SerializeField] private bool calibrated;
    [SerializeField] private string calibrationStage = "not run";
    [SerializeField] private int leftTopCount;
    [SerializeField] private int rightTopCount;
    [SerializeField] private int suppressedByRefractory;
    [SerializeField] private int suppressedByAlternation;
    [SerializeField] private int suppressedByAmplitude;
    [SerializeField] private string lastDetectedLeg = "";
    [SerializeField] private double lastDetectedTime;
    [SerializeField] private float[] currentSwingRate;
    [SerializeField] private float[] lastSwingAmplitude;

    public bool Calibrated => calibrated;
    public string CalibrationStage => calibrationStage;
    public int LeftTopCount => leftTopCount;
    public int RightTopCount => rightTopCount;
    public int SuppressedByRefractory => suppressedByRefractory;
    public int SuppressedByAlternation => suppressedByAlternation;

    // Per-board running state.
    private Vector3[] gyroBias;
    private float[] previousRate;
    private bool[] hasPreviousRate;
    private float[] positivePeakSinceCross;
    private float[] negativePeakSinceCross;
    private double[] nextAllowedEventTime;
    private bool[] startGoingUpArmed;

    private PeakLegSide? lastSubmittedLeg;
    private int sequenceCounter;
    private int boardCount;
    private bool subscribed;

    // Calibration accumulators, only non-null while the routine runs.
    private Vector3[] calibrationBiasSum;
    private int[] calibrationBiasSamples;
    private float[,] calibrationAxisSum;
    private float[,] calibrationAxisSumSq;
    private float[,] calibrationAxisSumCube;
    private float[,] calibrationAxisMax;
    private int[] calibrationWalkSamples;
    private bool calibrationRunning;
    private int calibrationPhase; // 1 = still, 2 = walking
    private bool detectorEnabledBeforeCalibration;

    private void Awake()
    {
        if (imuSource == null)
        {
            imuSource = FindFirstObjectByType<imu_data_receive>();
        }

        if (jasmineAdapter == null)
        {
            jasmineAdapter = FindFirstObjectByType<JasmineInputAdapter>();
        }
    }

    private void OnEnable()
    {
        if (imuSource == null)
        {
            Debug.LogWarning(
                "[ImuGaitEventDetector] No imu_data_receive assigned; detector is inert.",
                this);

            return;
        }

        // imu_data_receive sizes its boards in Start, so read the count lazily in the first
        // sample rather than here -- script execution order between the two is not guaranteed.
        imuSource.SampleFused += HandleSampleFused;
        subscribed = true;
    }

    private void OnDisable()
    {
        if (subscribed && imuSource != null)
        {
            imuSource.SampleFused -= HandleSampleFused;
        }

        subscribed = false;
    }

    // ===== Per-sample detection =====
    // Runs on the Unity main thread; imu_data_receive raises SampleFused from its Update.
    private void HandleSampleFused(
        imu_data_receive.ImuSampleSnapshot sample)
    {
        int board = sample.Board;

        EnsureBoardState(board + 1);

        float rate = ResolveSwingRate(board, sample.AngularVelocity);
        currentSwingRate[board] = rate;

        if (calibrationRunning)
        {
            AccumulateCalibration(board, sample.AngularVelocity);
            return;
        }

        TrackSwingPeaks(board, rate);

        if (!hasPreviousRate[board])
        {
            previousRate[board] = rate;
            hasPreviousRate[board] = true;
            return;
        }

        float previous = previousRate[board];
        previousRate[board] = rate;

        double now = Time.realtimeSinceStartupAsDouble;

        // Negative-going crossing: flexion has just given way to extension, so the thigh is at
        // its highest. This is Top -- the event the gait controller actually consumes.
        if (previous > 0f && rate <= 0f)
        {
            float swingAmplitude = positivePeakSinceCross[board];
            lastSwingAmplitude[board] = swingAmplitude;
            positivePeakSinceCross[board] = 0f;
            startGoingUpArmed[board] = false;

            TryEmit(
                board,
                JasmineLegEventType.Top,
                swingAmplitude,
                now,
                sample.Sequence);

            return;
        }

        // Positive-going crossing: extension has given way to flexion, so the thigh is at its
        // furthest back. Observation-only downstream.
        if (previous <= 0f && rate > 0f)
        {
            float swingAmplitude = negativePeakSinceCross[board];
            negativePeakSinceCross[board] = 0f;
            startGoingUpArmed[board] = true;

            if (emitHitBottom)
            {
                TryEmit(
                    board,
                    JasmineLegEventType.HitBottom,
                    swingAmplitude,
                    now,
                    sample.Sequence);
            }

            return;
        }

        // The swing is unambiguously under way: past the bottom and above threshold, but not yet
        // at the top. Observation-only downstream.
        if (emitStartGoingUp &&
            startGoingUpArmed[board] &&
            rate > minSwingRate)
        {
            startGoingUpArmed[board] = false;

            TryEmit(
                board,
                JasmineLegEventType.StartGoingUp,
                rate,
                now,
                sample.Sequence);
        }
    }

    // Peak magnitude seen on each side of zero since the last crossing. Comparing the completed
    // half-swing against minSwingRate is what separates a real step from sway, and it is more
    // stable than thresholding the instantaneous rate.
    private void TrackSwingPeaks(int board, float rate)
    {
        if (rate > positivePeakSinceCross[board])
        {
            positivePeakSinceCross[board] = rate;
        }

        if (-rate > negativePeakSinceCross[board])
        {
            negativePeakSinceCross[board] = -rate;
        }
    }

    private float ResolveSwingRate(
        int board,
        Vector3 angularVelocity)
    {
        GyroAxis axis = board < boardGyroAxis.Length
            ? boardGyroAxis[board]
            : GyroAxis.X;

        float raw = axis switch
        {
            GyroAxis.X => angularVelocity.x,
            GyroAxis.Y => angularVelocity.y,
            _ => angularVelocity.z
        };

        Vector3 bias = gyroBias[board];

        float biasComponent = axis switch
        {
            GyroAxis.X => bias.x,
            GyroAxis.Y => bias.y,
            _ => bias.z
        };

        float corrected = raw - biasComponent;

        bool invert = board < boardInvert.Length && boardInvert[board];

        return invert ? -corrected : corrected;
    }

    // ===== Gating and submission =====
    // Every rejection is counted rather than silently dropped, so a misbehaving detector is
    // distinguishable from a misbehaving controller during a trial.
    private void TryEmit(
        int board,
        JasmineLegEventType eventType,
        float swingAmplitude,
        double now,
        uint sourceSampleSequence)
    {
        if (!enableDetector)
        {
            return;
        }

        PeakLegSide leg = ResolveLeg(board);

        if (eventType == JasmineLegEventType.Top)
        {
            if (swingAmplitude < minSwingRate)
            {
                suppressedByAmplitude++;
                return;
            }

            if (now < nextAllowedEventTime[board])
            {
                suppressedByRefractory++;
                return;
            }

            if (enforceAlternation &&
                lastSubmittedLeg.HasValue &&
                lastSubmittedLeg.Value == leg)
            {
                suppressedByAlternation++;

                Debug.LogWarning(
                    $"[ImuGaitEventDetector] Consecutive {leg} Top suppressed. " +
                    "Check board-to-leg mapping and swing axis if this repeats.",
                    this);

                return;
            }

            nextAllowedEventTime[board] = now + minEventInterval;
            lastSubmittedLeg = leg;
            lastDetectedLeg = leg.ToString();
            lastDetectedTime = now;

            if (leg == PeakLegSide.Left)
            {
                leftTopCount++;
            }
            else
            {
                rightTopCount++;
            }
        }

        if (jasmineAdapter == null)
        {
            Debug.LogWarning(
                "[ImuGaitEventDetector] No JasmineInputAdapter assigned; event dropped.",
                this);

            return;
        }

        jasmineAdapter.SubmitSemanticEvent(
            leg,
            eventType,
            now,
            sequenceCounter++);
    }

    private PeakLegSide ResolveLeg(int board)
    {
        if (board < boardToLeg.Length)
        {
            return boardToLeg[board];
        }

        // A board with no mapping would otherwise silently drive the left leg.
        Debug.LogWarning(
            $"[ImuGaitEventDetector] Board {board} has no boardToLeg entry; defaulting to Left.",
            this);

        return PeakLegSide.Left;
    }

    // ===== Calibration =====
    // Two phases, because the two unknowns need different conditions: gyro bias needs stillness,
    // while the swing axis and its sign only reveal themselves during walking.
    [ContextMenu("Calibrate Detector")]
    public void CalibrateDetector()
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning(
                "[ImuGaitEventDetector] Calibration needs Play Mode and a live IMU stream.",
                this);

            return;
        }

        if (calibrationRunning)
        {
            return;
        }

        StartCoroutine(CalibrationRoutine());
    }

    private IEnumerator CalibrationRoutine()
    {
        EnsureBoardState(Mathf.Max(boardCount, boardToLeg.Length));

        calibrationRunning = true;
        calibrated = false;
        detectorEnabledBeforeCalibration = enableDetector;
        enableDetector = false;

        calibrationBiasSum = new Vector3[boardCount];
        calibrationBiasSamples = new int[boardCount];
        calibrationAxisSum = new float[boardCount, 3];
        calibrationAxisSumSq = new float[boardCount, 3];
        calibrationAxisSumCube = new float[boardCount, 3];
        calibrationAxisMax = new float[boardCount, 3];
        calibrationWalkSamples = new int[boardCount];

        calibrationPhase = 1;
        calibrationStage = "Phase 1/2: STAND STILL";
        Debug.Log(
            $"[ImuGaitEventDetector] Calibration phase 1: stand still for " +
            $"{stillCalibrationSeconds:F1}s.",
            this);

        yield return new WaitForSecondsRealtime(stillCalibrationSeconds);

        for (int board = 0; board < boardCount; board++)
        {
            if (calibrationBiasSamples[board] > 0)
            {
                gyroBias[board] =
                    calibrationBiasSum[board] / calibrationBiasSamples[board];
            }
            else
            {
                Debug.LogWarning(
                    $"[ImuGaitEventDetector] Board {board} sent no samples during the still " +
                    "phase; its bias is unchanged. Check packetsPerSecond.",
                    this);
            }
        }

        calibrationPhase = 2;
        calibrationStage = "Phase 2/2: WALK NORMALLY";
        Debug.Log(
            $"[ImuGaitEventDetector] Calibration phase 2: walk normally for " +
            $"{walkCalibrationSeconds:F1}s.",
            this);

        // The bias is now known, so the walk phase accumulates bias-corrected statistics.
        yield return new WaitForSecondsRealtime(walkCalibrationSeconds);

        ResolveCalibrationResults();

        calibrationRunning = false;
        calibrationPhase = 0;
        calibrationBiasSum = null;
        calibrationBiasSamples = null;
        calibrationAxisSum = null;
        calibrationAxisSumSq = null;
        calibrationAxisSumCube = null;
        calibrationAxisMax = null;
        calibrationWalkSamples = null;

        ResetRunningState();
        enableDetector = detectorEnabledBeforeCalibration;
    }

    private void AccumulateCalibration(
        int board,
        Vector3 angularVelocity)
    {
        if (board >= boardCount || calibrationAxisSum == null)
        {
            return;
        }

        if (calibrationPhase == 1)
        {
            calibrationBiasSum[board] += angularVelocity;
            calibrationBiasSamples[board]++;
            return;
        }

        Vector3 corrected = angularVelocity - gyroBias[board];

        for (int axis = 0; axis < 3; axis++)
        {
            float value = corrected[axis];

            calibrationAxisSum[board, axis] += value;
            calibrationAxisSumSq[board, axis] += value * value;
            calibrationAxisSumCube[board, axis] += value * value * value;

            float magnitude = Mathf.Abs(value);

            if (magnitude > calibrationAxisMax[board, axis])
            {
                calibrationAxisMax[board, axis] = magnitude;
            }
        }

        calibrationWalkSamples[board]++;
    }

    private void ResolveCalibrationResults()
    {
        EnsureInspectorArrayLengths();

        float globalPeakRate = 0f;
        bool anyBoardResolved = false;

        for (int board = 0; board < boardCount; board++)
        {
            int samples = calibrationWalkSamples[board];

            if (samples < 20)
            {
                Debug.LogWarning(
                    $"[ImuGaitEventDetector] Board {board} produced only {samples} walk " +
                    "samples; its axis was not changed.",
                    this);

                continue;
            }

            // The medio-lateral axis is the one the swing actually happens about, so it carries
            // by far the most variance during walking.
            int bestAxis = 0;
            float bestVariance = -1f;

            for (int axis = 0; axis < 3; axis++)
            {
                float mean = calibrationAxisSum[board, axis] / samples;
                float variance =
                    calibrationAxisSumSq[board, axis] / samples - mean * mean;

                if (variance > bestVariance)
                {
                    bestVariance = variance;
                    bestAxis = axis;
                }
            }

            // Sign is not recoverable from variance -- flexion and extension are symmetric in it.
            // Gait is not symmetric in TIME though: swing is fast and brief, stance is slow and
            // long, so the large-magnitude tail of the rate distribution belongs to flexion.
            // Positive skew therefore means the axis already reads flexion as positive.
            float axisMean = calibrationAxisSum[board, bestAxis] / samples;
            float axisVariance =
                calibrationAxisSumSq[board, bestAxis] / samples - axisMean * axisMean;
            float standardDeviation = Mathf.Sqrt(Mathf.Max(1e-9f, axisVariance));

            float thirdMoment =
                calibrationAxisSumCube[board, bestAxis] / samples
                - 3f * axisMean * calibrationAxisSumSq[board, bestAxis] / samples
                + 2f * axisMean * axisMean * axisMean;

            float skewness =
                thirdMoment / (standardDeviation * standardDeviation * standardDeviation);

            boardGyroAxis[board] = (GyroAxis)bestAxis;
            boardInvert[board] = skewness < 0f;

            float peakRate = calibrationAxisMax[board, bestAxis];
            globalPeakRate = Mathf.Max(globalPeakRate, peakRate);
            anyBoardResolved = true;

            Debug.Log(
                $"[ImuGaitEventDetector] Board {board} ({ResolveLeg(board)}): " +
                $"axis={boardGyroAxis[board]}, invert={boardInvert[board]}, " +
                $"peak={peakRate:F3} rad/s, sd={standardDeviation:F3}, " +
                $"skew={skewness:F3}, samples={samples}.",
                this);
        }

        if (anyBoardResolved && globalPeakRate > 0f)
        {
            minSwingRate = Mathf.Max(0.05f, globalPeakRate * thresholdFraction);
            calibrated = true;
            calibrationStage = "done";

            Debug.Log(
                $"[ImuGaitEventDetector] Calibration complete. minSwingRate={minSwingRate:F3} rad/s. " +
                "If a leg is inverted in play, flip its boardInvert entry.",
                this);
        }
        else
        {
            calibrationStage = "failed: no usable walk data";

            Debug.LogWarning(
                "[ImuGaitEventDetector] Calibration produced no usable walk data. " +
                "Check that both boards are streaming before retrying.",
                this);
        }
    }

    // ===== State management =====
    private void EnsureBoardState(int requiredCount)
    {
        if (requiredCount <= boardCount && gyroBias != null)
        {
            return;
        }

        int newCount = Mathf.Max(requiredCount, boardCount);

        Resize(ref gyroBias, newCount);
        Resize(ref previousRate, newCount);
        Resize(ref hasPreviousRate, newCount);
        Resize(ref positivePeakSinceCross, newCount);
        Resize(ref negativePeakSinceCross, newCount);
        Resize(ref nextAllowedEventTime, newCount);
        Resize(ref startGoingUpArmed, newCount);
        Resize(ref currentSwingRate, newCount);
        Resize(ref lastSwingAmplitude, newCount);

        boardCount = newCount;

        EnsureInspectorArrayLengths();
    }

    // The three per-board Inspector arrays are authored by hand and can easily end up shorter
    // than the board count. Growing them keeps indexing safe without discarding what was set.
    private void EnsureInspectorArrayLengths()
    {
        if (boardToLeg.Length < boardCount)
        {
            PeakLegSide[] grown = new PeakLegSide[boardCount];
            boardToLeg.CopyTo(grown, 0);
            boardToLeg = grown;
        }

        if (boardGyroAxis.Length < boardCount)
        {
            GyroAxis[] grown = new GyroAxis[boardCount];
            boardGyroAxis.CopyTo(grown, 0);
            boardGyroAxis = grown;
        }

        if (boardInvert.Length < boardCount)
        {
            bool[] grown = new bool[boardCount];
            boardInvert.CopyTo(grown, 0);
            boardInvert = grown;
        }
    }

    private static void Resize<T>(ref T[] array, int count)
    {
        T[] grown = new T[count];

        if (array != null)
        {
            array.CopyTo(grown, 0);
        }

        array = grown;
    }

    private void ResetRunningState()
    {
        for (int board = 0; board < boardCount; board++)
        {
            previousRate[board] = 0f;
            hasPreviousRate[board] = false;
            positivePeakSinceCross[board] = 0f;
            negativePeakSinceCross[board] = 0f;
            nextAllowedEventTime[board] = 0.0;
            startGoingUpArmed[board] = false;
        }

        lastSubmittedLeg = null;
    }

    [ContextMenu("Reset Detector Diagnostics")]
    public void ResetDiagnostics()
    {
        leftTopCount = 0;
        rightTopCount = 0;
        suppressedByRefractory = 0;
        suppressedByAlternation = 0;
        suppressedByAmplitude = 0;
        lastDetectedLeg = "";
        lastDetectedTime = 0.0;
        sequenceCounter = 0;

        ResetRunningState();
    }
}
