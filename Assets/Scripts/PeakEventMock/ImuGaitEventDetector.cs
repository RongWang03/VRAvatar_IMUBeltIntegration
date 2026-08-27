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
    /// <summary>One physical footfall, de-duplicated across boards. Measured, not predicted.</summary>
    public readonly struct MeasuredFootfall
    {
        public MeasuredFootfall(double unityTimestamp, int firstBoard, float shock)
        {
            UnityTimestamp = unityTimestamp;
            FirstBoard = firstBoard;
            Shock = shock;
        }

        public double UnityTimestamp { get; }
        public int FirstBoard { get; }
        public float Shock { get; }
    }

    /// <summary>
    /// Raised once per footfall. Carries no leg: both hip boards feel both feet, so this
    /// component can say WHEN a foot landed but never WHICH.
    /// </summary>
    public event System.Action<MeasuredFootfall> FootfallMeasured;

    public enum AttributionMethod
    {
        Angle,
        Timing,

        /// <summary>Take the leg from LegAttributionBySwingDominance instead of deciding here.
        /// The only method with a measured accuracy worth trusting; Angle and Timing are kept
        /// for comparison only.</summary>
        SwingDominance
    }

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
    [Tooltip("Which leg each board observes. Verified on hardware 2026-08-19 by single-leg " +
             "lift test: board 0 = LEFT hip pocket, board 1 = RIGHT hip pocket. The board " +
             "physically labelled 'right' sits on board index 1, not 0.")]
    [SerializeField]
    private PeakLegSide[] boardToLeg =
    {
        PeakLegSide.Left,
        PeakLegSide.Right
    };

    [Tooltip("Which gyro axis is medio-lateral for each board. Depends on how the board sits in " +
             "the pocket, so it varies per wear -- run Calibrate Detector rather than guessing.")]
    [SerializeField]
    private GyroAxis[] boardGyroAxis =
    {
        GyroAxis.X,
        GyroAxis.X
    };

    [Tooltip("The direction each board actually rotates about during walking, in board frame. " +
             "Found by calibration as the principal component of the gyro, because a board sitting " +
             "at an angle in the pocket spreads the swing across all three axes -- picking a " +
             "single best axis then throws most of the signal away and flips between runs.")]
    [SerializeField] private Vector3[] boardSwingAxis;

    [Tooltip("Use boardSwingAxis instead of a single X/Y/Z axis. Falls back automatically when " +
             "calibration has not produced one.")]
    [SerializeField] private bool useSwingAxisVector = true;

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
    [Tooltip("rad/s. Fallback threshold, used for any board without a calibrated one of its own. " +
             "Prefer Min Swing Rate Per Board, which calibration fills in.")]
    [SerializeField] private float minSwingRate = 1.2f;

    [Tooltip("rad/s per board, set by calibration from THAT board's own peak. One shared " +
             "threshold taken from the loudest board silently starved the quieter one, which is " +
             "why one leg needed exaggerated swings to register.")]
    [SerializeField] private float[] minSwingRatePerBoard;

    [Min(0f)]
    [Tooltip("Seconds. Per-leg refractory period; suppresses double-fires on a noisy stride. " +
             "Must stay below the fastest same-leg cycle the controller accepts (0.7 s).")]
    [SerializeField] private float minEventInterval = 0.35f;

    [Tooltip("Reject a Top for the same leg twice in a row. The gait controller enforces strict " +
             "alternation anyway, but rejecting here makes a double-fire visible as a detector " +
             "counter instead of an unexplained stutter.")]
    [SerializeField] private bool enforceAlternation = true;

    [Min(1)]
    [Tooltip("Rank used as the calibration peak: 1 means the single largest sample (the old " +
             "behaviour), 20 means the 20th largest. A knock lasts 1-3 samples while a real " +
             "swing peak contributes several samples on every stride, so a rank of about 20 " +
             "reads the walk and ignores the knock. Two thresholds were poisoned this way in " +
             "one session -- 4.89 rad/s silenced a board outright, 2.24 starved it.")]
    [SerializeField] private int calibrationPeakRank = 20;

    [Header("Processing Gate")]
    [Tooltip("Hold every event back until the wearer presses the start button. Signal " +
             "conditioning keeps running the whole time -- only the OUTPUT is gated -- so the " +
             "drift filter and shock baseline are already settled when processing begins. " +
             "Gating the filters as well would give every session a cold-start transient on " +
             "exactly the first steps.")]
    [SerializeField] private bool requireStartPress = true;

    [Tooltip("Whether the controller can start and stop processing. Independent of the " +
             "calibration trigger's own controller setting, so either can be turned off alone.")]
    [SerializeField] private bool allowControllerStart = true;

    [SerializeField] private OVRInput.RawButton startButton = OVRInput.RawButton.A;
    [SerializeField] private OVRInput.Controller startController = OVRInput.Controller.RTouch;
    [SerializeField] private KeyCode startKey = KeyCode.Space;

    [Min(0.2f)]
    [Tooltip("Seconds of lockout after a press, so one physical press cannot start and stop.")]
    [SerializeField] private float startPressLockout = 0.6f;

    [Tooltip("Zero the counters each time processing starts, so every run is measured from " +
             "scratch without exiting Play.")]
    [SerializeField] private bool resetCountersOnStart = true;

    [Tooltip("Live state. Nothing is emitted, logged or counted while this is false.")]
    [SerializeField] private bool processingActive;

    [Header("Crossing Hysteresis")]
    [Tooltip("Require the swing rate to commit past a band either side of zero before a " +
             "crossing counts, instead of firing on any sign change. A bare sign test treats " +
             "zero as a knife edge: standing still produced about 30 crossings in 5.5 seconds, " +
             "and a mid-swing dip split one swing into amplitudes 0.50 and 0.11 because the " +
             "peak accumulator resets at every crossing.")]
    [SerializeField] private bool useCrossingHysteresis = true;

    [Min(0.0f)]
    [Tooltip("Half-width of the dead zone, rad/s. Standing noise measured 0.00-0.12 and real " +
             "swings 0.5-2.4, so 0.15 clears the noise by a wide margin without coming near a " +
             "real swing. Zero disables the band and reproduces the old sign test exactly.")]
    [SerializeField] private float crossingHysteresis = 0.15f;

    [Header("Cross-Board Gate (Top)")]
    [Tooltip("Replace the absolute swing threshold with a comparison against the OTHER board. " +
             "A stance thigh rotates backwards while a swinging thigh rotates forwards, so the " +
             "comparison holds at any pace -- whereas an absolute threshold has to be re-placed " +
             "every time the pace changes, and four consecutive walks measured 56% to 175% " +
             "recall on one setting. Leave OFF for the first run and read the logged ratios.")]
    [SerializeField] private bool useCrossBoardGate = false;

    [Min(0.05f)]
    [Tooltip("Seconds of swing-rate history compared at each candidate Top. Long enough to hold " +
             "the flexion that just ended, short enough not to reach into the previous step.")]
    [SerializeField] private float crossBoardWindowSeconds = 0.30f;

    [Min(1.0f)]
    [Tooltip("How much larger this board's flexion peak must be than the other board's. On raw " +
             "magnitude this separated barely at all (~1.0); on the projected rate the same steps " +
             "measured about 2:1. Set it from the logged numbers, not from this default.")]
    [SerializeField] private float crossBoardMinRatio = 1.5f;

    [Min(0.05f)]
    [Tooltip("Absolute floor, in rad/s of projected flexion rate. Its only job is to answer: is " +
             "anything moving at all. Standing noise can satisfy any ratio. Keep it far below a " +
             "real swing so it never becomes the operating threshold.")]
    [SerializeField] private float crossBoardFloor = 0.5f;

    [Tooltip("Log the comparison at every candidate Top, whether or not the gate is in use. " +
             "This is how the ratio is measured before it is trusted.")]
    [SerializeField] private bool logCrossBoardRatios = true;

    [Header("Rate Drift Correction")]
    [Tooltip("Detect Top against the rate's own slow average instead of absolute zero. Gyro bias " +
             "drifts with temperature after calibration; once the rate sits above zero the " +
             "negative-going crossing stops happening, so Top goes missing and the integrated " +
             "angle runs away in one direction instead of returning each stride.")]
    [SerializeField] private bool removeRateDrift = true;

    [Min(0.5f)]
    [Tooltip("Seconds. Must span several strides, so it tracks bias drift but not the swing.")]
    [SerializeField] private float rateDriftTau = 3f;

    [Tooltip("Read-only: the offset currently being subtracted from each board's rate.")]
    [SerializeField] private float[] rateDrift;

    [Header("Foot Contact (observation only)")]
    [Tooltip("Detect heel strike from the accelerometer shock. Drives nothing -- it exists to " +
             "measure how far the gait controller's PREDICTED landing sits from the real one.")]
    [SerializeField] private bool detectFootContact = false;

    [Min(0.1f)]
    [Tooltip("m/s^2 above the running baseline. Heel strike at the hip is damped compared with " +
             "a shank sensor, so this sits well below the raw impact figure.")]
    [SerializeField] private float contactShockThreshold = 2.5f;

    [Min(0.05f)]
    [Tooltip("Seconds. Refractory per board -- one strike rings for several samples.")]
    [SerializeField] private float contactMinInterval = 0.25f;

    [Range(0.1f, 0.95f)]
    [Tooltip("Hysteresis. The shock must fall below this FRACTION of the threshold before the " +
             "board can fire again. A footfall is not one impulse -- heel strike, foot flat and " +
             "weight transfer make it ring, so re-arming at the same level the detector fires " +
             "at retriggers on the ripple and produces several contacts per step.")]
    [SerializeField] private float contactReleaseFraction = 0.5f;

    [Min(0.02f)]
    [Tooltip("Seconds. Time constant of the baseline the shock is measured against. Must be well " +
             "above a strike (~0.03 s) and well below a stride, so the baseline tracks posture " +
             "and gravity but never the impact itself.")]
    [SerializeField] private float contactBaselineTau = 0.30f;

    [Tooltip("Print every contact to the Console. Useful while tuning the threshold, noisy after.")]
    [SerializeField] private bool logFootContact = true;

    [Tooltip("Periodically report the highest shock seen since the last report. The Inspector " +
             "redraws at frame rate and a strike lasts a few samples, so watching Current Shock " +
             "by eye systematically under-reads the peak. This sees every sample.")]
    [SerializeField] private bool logShockPeaks = false;

    [Min(0.25f)]
    [Tooltip("Seconds between peak reports.")]
    [SerializeField] private float shockPeakReportInterval = 2f;

    [Tooltip("Click once per footfall. Both boards feel every step, so the beep is de-duplicated " +
             "across boards -- otherwise you would hear two clicks per step, not one.")]
    [SerializeField] private bool beepOnFootContact = false;

    [Min(100f)]
    [Tooltip("Hz. Used when the leg is unknown -- Attribute Contact Leg off, or no verdict yet.")]
    [SerializeField] private float contactBeepFrequency = 1600f;

    [Tooltip("Pitch the contact beep by which leg was attributed, so left and right are " +
             "distinguishable by ear. Needs Attribute Contact Leg on.")]
    [SerializeField] private bool pitchBeepByLeg = false;

    [Tooltip("OFF by default: every footfall gets a left or right tone, so the beep always says " +
             "which leg. Turn ON only if you want ambiguous verdicts to fall back to the neutral " +
             "tone -- useful for hearing where the method is unsure, not for normal use.")]
    [SerializeField] private bool requireConfidentVerdictForPitch = false;

    [Min(0.005f)]
    [Tooltip("Seconds. A pushed verdict older than this is treated as belonging to a previous " +
             "footfall and the neutral tone is used. Same-frame delivery means this only has to " +
             "cover one frame.")]
    [SerializeField] private float externalVerdictMaxAgeSeconds = 0.05f;

    [Tooltip("Which attribution drives the pitch. Switch between them across two walks to hear " +
             "where they differ, rather than reading it out of the Console afterwards.")]
    [SerializeField] private AttributionMethod beepPitchSource = AttributionMethod.SwingDominance;

    [Min(100f)]
    [Tooltip("Hz for LEFT. Low = Left is the mnemonic; an octave below the right tone.")]
    [SerializeField] private float contactBeepFrequencyLeft = 1000f;

    [Min(100f)]
    [Tooltip("Hz for RIGHT. An octave above left, which is about the easiest interval to call " +
             "correctly at speed without training.")]
    [SerializeField] private float contactBeepFrequencyRight = 2000f;

    [Min(0.01f)]
    [Tooltip("Seconds. Keep this short -- footfalls arrive about twice a second and a long tone " +
             "smears into the next one, which destroys the timing cue you are listening for.")]
    [SerializeField] private float contactBeepDuration = 0.04f;

    [Tooltip("Reject a contact that arrives too soon relative to the wearer's OWN recent step " +
             "rhythm, rather than against a fixed number of seconds. Duplicates were measured at " +
             "0.50-0.56 s against real steps at 0.72-0.87 s -- the two overlap badly as absolute " +
             "values across different walking speeds, but a duplicate is always a fraction of a " +
             "stride, whatever the stride happens to be.")]
    [SerializeField] private bool adaptiveFootfallSpacing = true;

    [Range(0.3f, 0.95f)]
    [Tooltip("A contact must be at least this fraction of the recent mean step interval. At the " +
             "measured 0.80 s mean, 0.70 puts the bar at 0.56 s -- above the duplicates, below " +
             "the real steps -- and it moves with cadence instead of needing retuning.")]
    [SerializeField] private float adaptiveSpacingFraction = 0.70f;

    [Min(0f)]
    [Tooltip("Running mean interval between accepted footfalls. SEEDED, not left at zero: the " +
             "gate has no cadence estimate on the very first interval of a session, and a " +
             "duplicate arriving there would be judged against the fixed floor alone and let " +
             "through. Set it near your usual step interval; it adapts from the first stride.")]
    [SerializeField] private float meanFootfallInterval = 0.8f;

    [Min(1)]
    [Tooltip("Consecutive rejections after which the gate gives up and accepts, reseeding the " +
             "cadence estimate from that interval. Without this, a seed far above the wearer's " +
             "real cadence would reject every step forever -- rejections never update the mean, " +
             "so nothing would correct it.")]
    [SerializeField] private int spacingRejectionsBeforeReset = 3;

    [Min(0f)]
    [Tooltip("Seconds. Contacts closer together than this are treated as the same footfall seen " +
             "by both boards, and beep once. Measured pair spread was 1-122 ms; the margin above " +
             "that covers a board firing late on the ring-down. Must stay below the shortest " +
             "real gap between footfalls, measured at 0.32 s.")]
    [SerializeField] private float contactBeepDedupeWindow = 0.22f;

    [Header("Raw Signal Report")]
    [Tooltip("Periodically report the min/max swing rate and integrated angle per board. This is " +
             "the signal every other feature is derived from -- when Top counts, angles and " +
             "contacts disagree with each other, read this before changing any threshold.")]
    [SerializeField] private bool logSwingRateRange = false;

    [Min(0.25f)]
    [SerializeField] private float swingRateReportInterval = 2f;

    [Header("Leg Attribution (observation only)")]
    [Tooltip("At each footfall, work out WHICH leg landed, by two independent methods, and " +
             "record how often they agree. Drives nothing. Both methods read each board's own " +
             "gyro -- unlike the shock, which crosses the pelvis and cannot separate the legs.")]
    [SerializeField] private bool attributeContactLeg = false;

    [Tooltip("Print one line per footfall with both verdicts.")]
    [SerializeField] private bool logLegAttribution = true;

    [Min(0f)]
    [Tooltip("Seconds. The gap between the two boards' time-since-Top must exceed this for the " +
             "verdict to count as confident. Expected separation is roughly 0.25 s vs 0.8 s, so " +
             "anything under ~0.15 s means the two legs are not distinguishable on that stride.")]
    [SerializeField] private float attributionMinMargin = 0.15f;

    [Tooltip("Read-only: gap between the two boards' time-since-Top at the last footfall.")]
    [SerializeField] private float lastAttributionMargin;

    [SerializeField] private int attributionConfidentCount;
    [SerializeField] private int attributionAmbiguousCount;

    [Header("Optional Event Types")]
    [Tooltip("HitBottom is observation-only downstream and roughly doubles the log volume.")]
    [SerializeField] private bool emitHitBottom = false;

    [Tooltip("StartGoingUp is observation-only downstream.")]
    [SerializeField] private bool emitStartGoingUp = false;

    [Header("Calibration Persistence")]
    [Tooltip("Save calibration and reload it on the next Play. Unity discards runtime changes to " +
             "serialized fields when Play ends, so without this every session silently starts " +
             "uncalibrated and falls back to the shared threshold.")]
    [SerializeField] private bool persistCalibration = true;

    [Tooltip("PlayerPrefs key prefix. Change it to keep separate calibrations per wearer.")]
    [SerializeField] private string calibrationKey = "ImuGait";

    [Header("Calibration Trigger")]
    [Tooltip("Start calibration from the headset, so the wearer can run it without reaching the " +
             "Inspector -- which matters because the wearer has to be standing and then walking.")]
    [SerializeField] private bool allowControllerCalibration = true;

    [Tooltip("Held, not tapped. Calibration takes 40 s and forces the detector off while it " +
             "runs, so an accidental press mid-trial would be expensive.")]
    [SerializeField] private OVRInput.RawButton calibrateButton = OVRInput.RawButton.X;

    [SerializeField] private OVRInput.Controller calibrateController = OVRInput.Controller.LTouch;

    [Min(0f)]
    [Tooltip("Seconds the button must be held before calibration starts.")]
    [SerializeField] private float calibrateHoldSeconds = 1.5f;

    [SerializeField] private string calibrateHoldStatus = "";

    [Tooltip("Live OVR input state. If this stays 'no controller', OVRInput is not seeing the " +
             "hardware at all and the button choice is irrelevant.")]
    [SerializeField] private string calibrateInputDebug = "";

    [Tooltip("Also start calibration from the keyboard, for desk testing without the headset.")]
    [SerializeField] private bool allowKeyboardCalibration = true;

    [SerializeField] private KeyCode calibrateKey = KeyCode.C;

    [Header("Calibration")]
    [Min(0.5f)]
    [Tooltip("Seconds of standing still used to measure gyro bias. Longer is strictly better " +
             "here: bias is a mean, so its error falls with the square root of the sample count.")]
    [SerializeField] private float stillCalibrationSeconds = 15f;

    [Min(0f)]
    [Tooltip("Seconds of dead time between the still and walk phases, so the wearer can start " +
             "moving without those transition samples polluting either measurement. Nothing is " +
             "recorded during this window.")]
    [SerializeField] private float transitionSeconds = 10f;

    [Min(1f)]
    [Tooltip("Seconds of walking used to pick the swing axis and set the threshold.")]
    [SerializeField] private float walkCalibrationSeconds = 15f;

    [Header("Calibration Audio")]
    [Tooltip("Beep at the start and end of each calibration phase. The tone is generated at " +
             "runtime, so no audio asset is needed.")]
    [SerializeField] private bool calibrationBeeps = true;

    [Range(0f, 1f)]
    [SerializeField] private float beepVolume = 0.5f;

    [Tooltip("Hz. The start-of-phase beep. The end-of-phase beep is one octave below it, so " +
             "\"go\" and \"stop\" are distinguishable without looking at the Inspector.")]
    [SerializeField] private float beepStartFrequency = 880f;

    [Min(0.02f)]
    [Tooltip("Seconds. Duration of each beep.")]
    [SerializeField] private float beepDuration = 0.15f;

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
    [SerializeField] private int suppressedByCrossBoard;
    [SerializeField] private float lastCrossBoardRatio;
    [SerializeField] private int suppressedByAmplitude;
    [SerializeField] private string lastDetectedLeg = "";
    [SerializeField] private double lastDetectedTime;
    [SerializeField] private float[] currentSwingRate;
    [SerializeField] private float[] lastSwingAmplitude;
    [SerializeField] private int[] contactCount;
    [SerializeField] private float[] lastContactShock;
    [SerializeField] private float[] currentShock;

    [Header("Internal Latency Probe")]
    [Tooltip("Measures the delay this software adds between a packet arriving at the PC and the " +
             "footfall event being raised. It CANNOT see the physical half -- the gap between " +
             "the heel actually touching the floor and the shock crossing threshold -- which " +
             "needs an outside instrument. Observation-only.")]
    [SerializeField] private bool measureInternalLatency = true;

    [Min(1)]
    [Tooltip("Log a running summary every N measured footfalls.")]
    [SerializeField] private int latencyReportEvery = 10;

    [Tooltip("Milliseconds from the receive thread parsing the packet to the footfall event " +
             "being raised. Covers queueing, the frame boundary and fusion.")]
    [SerializeField] private float lastQueueMs;

    [SerializeField] private float meanQueueMs;
    [SerializeField] private float worstQueueMs;

    [Tooltip("Milliseconds between the rising edge that fired and the actual peak of the shock. " +
             "Positive means the detector fired BEFORE the peak, which is intended -- the onset " +
             "is the physically meaningful instant.")]
    [SerializeField] private float lastPeakLeadMs;

    [SerializeField] private float meanPeakLeadMs;

    [Tooltip("Milliseconds spent inside the beep call itself. Does NOT include the headset's " +
             "own audio output delay, which no code on this side can observe.")]
    [SerializeField] private float lastBeepCallMs;

    [SerializeField] private int latencySamples;

    [Tooltip("Running |a| the shock is measured against. ~9.81 standing, a little above it " +
             "walking. If this drifts far from those, the board or its units are wrong.")]
    [SerializeField] private float[] shockBaseline;

    [Tooltip("Highest shock seen since the last peak report. This is the number to set the " +
             "threshold from -- not Current Shock, which you can only catch by luck.")]
    [SerializeField] private float[] peakShockSinceReport;

    [Tooltip("Degrees of thigh travel since this board's own Top. Near 0 = just left peak " +
             "flexion (leg swinging down to land). Large negative = long past it (leg trailing).")]
    [SerializeField] private float[] thighAngleSinceTop;

    [Tooltip("Seconds since this board's own Top.")]
    [SerializeField] private float[] timeSinceTop;

    [SerializeField] private string lastAttributionByAngle = "";
    [SerializeField] private string lastAttributionByTiming = "";
    [SerializeField] private int attributionAgreeCount;
    [SerializeField] private int attributionDisagreeCount;

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

    // Foot-contact state. Separate from the swing detector: different signal, different gate.

    private bool[] hasRateDrift;
    private bool[] hasShockBaseline;
    private bool[] contactArmed;
    private double nextShockReportTime;
    private double nextSwingRateReportTime;
    private float[,] axisMinSinceReport;
    private float[,] axisMaxSinceReport;
    private float[] rateMinSinceReport;
    private float[] rateMaxSinceReport;
    private float[] angleMinSinceReport;
    private float[] angleMaxSinceReport;
    private int[] topsSinceReport;
    private double lastFootfallRaisedTime = double.NegativeInfinity;
    private int consecutiveSpacingRejections;

    // Integrated from each board's own gyro and zeroed at that board's Top, so the integration
    // window is never longer than one gait cycle. That bound is what makes it usable where the
    // absolute fused orientation is not: drift has under a second to accumulate.
    private float[] thighAngleRad;
    private double[] lastTopTime;
    private double[] lastTopResetTime;
    private double lastAttributionTime = double.NegativeInfinity;
    private PeakLegSide? lastAttributedLeg;

    // Verdict pushed in by LegAttributionBySwingDominance. That component subscribes to
    // FootfallMeasured, which is raised a few lines before the beep, so its verdict for THIS
    // footfall already exists by the time the tone is chosen -- no extra frame of delay.
    private PeakLegSide? externalLeg;
    private bool externalConfident;
    private double externalVerdictTime = -1.0;

    // Contact beeps fire ~2x a second for the whole session, so the clip is cached rather than
    // synthesised each time -- a fresh float[] per footfall is avoidable GC pressure in VR.
    private readonly System.Collections.Generic.Dictionary<int, AudioClip> contactToneCache =
        new System.Collections.Generic.Dictionary<int, AudioClip>();
    private double[] nextAllowedContactTime;

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
    private float[,,] calibrationOuter;
    private float[,] calibrationAxisMax;

    // The largest calibrationPeakRank samples per board and axis, kept sorted descending. Taking
    // the smallest of these instead of the outright maximum is what makes the peak robust: one
    // spike occupies one slot and is pushed out of the way by the genuine repeated peaks.
    private float[,,] calibrationAxisTop;
    private int[] calibrationWalkSamples;
    private bool calibrationRunning;
    private int calibrationPhase; // 1 = still, 2 = walking, 3 = transition (recorded to neither)
    private AudioSource beepSource;
    private bool detectorEnabledBeforeCalibration;
    private float calibrateHeldFor;

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

        LoadCalibration();

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
        rate = RemoveRateDrift(board, rate, sample);
        currentSwingRate[board] = rate;

        if (calibrationRunning)
        {
            AccumulateCalibration(board, sample.AngularVelocity);
            return;
        }

        TrackCrossBoardRate(board, rate);
        TrackSwingPeaks(board, rate);
        IntegrateThighAngle(board, rate, sample);
        TrackFootContact(board, sample);

        if (!hasPreviousRate[board])
        {
            previousRate[board] = rate;
            hasPreviousRate[board] = true;
            return;
        }

        float previous = previousRate[board];
        previousRate[board] = rate;

        double now = Time.realtimeSinceStartupAsDouble;

        bool topCrossing;
        bool bottomCrossing;

        if (useCrossingHysteresis && crossingHysteresis > 0f)
        {
            float band = crossingHysteresis;

            if (!hasSwingState[board])
            {
                // Seed from whichever side the signal is already on. A board that starts inside
                // the band is treated as falling, so the first event it can produce is a Top --
                // never a spurious one from the seed itself.
                swingRising[board] = rate > band;
                hasSwingState[board] = true;
            }

            topCrossing = swingRising[board] && rate < -band;
            bottomCrossing = !swingRising[board] && rate > band;

            if (topCrossing)
            {
                swingRising[board] = false;
            }
            else if (bottomCrossing)
            {
                swingRising[board] = true;
            }
        }
        else
        {
            topCrossing = previous > 0f && rate <= 0f;
            bottomCrossing = previous <= 0f && rate > 0f;
        }

        // Negative-going crossing: flexion has just given way to extension, so the thigh is at
        // its highest. This is Top -- the event the gait controller actually consumes.
        if (topCrossing)
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

            // Two gates, and only two. Amplitude separates a swing peak from the gyro wobbling
            // across zero. Refractory is the other PHYSICAL constraint: a thigh cannot reach
            // peak flexion twice inside one refractory period, so a second crossing that close
            // is noise however large it looks.
            //
            // Alternation is deliberately NOT applied here. That one is policy about which
            // events the gait controller accepts -- the thigh really did peak, so it must still
            // move this reference, or a suppressed event would freeze that leg's phase and the
            // next attribution would read a stale time.
            bool amplitudeOk = swingAmplitude >= ResolveMinSwingRate(board);
            bool spacingOk = now - lastTopResetTime[board] >= minEventInterval;

            if (amplitudeOk && spacingOk)
            {
                lastTopResetTime[board] = now;
                thighAngleRad[board] = 0f;
                lastTopTime[board] = now;
                topsSinceReport[board]++;
            }

            return;
        }

        // Positive-going crossing: extension has given way to flexion, so the thigh is at its
        // furthest back. Observation-only downstream.
        if (bottomCrossing)
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
            rate > ResolveMinSwingRate(board))
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
        if (useSwingAxisVector &&
            boardSwingAxis != null &&
            board < boardSwingAxis.Length &&
            boardSwingAxis[board].sqrMagnitude > 0.5f)
        {
            // Sign is already baked into the stored axis direction, so no invert flag here.
            return Vector3.Dot(angularVelocity - gyroBias[board], boardSwingAxis[board]);
        }

        return ResolveSwingRateLegacy(board, angularVelocity);
    }

    private float ResolveSwingRateLegacy(
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

    // ===== Foot contact =====
    // Heel strike is an IMPACT, so it lives in the accelerometer, not the gyro. The swing
    // detector cannot see it: at contact the thigh is mid-extension with no turning point, which
    // is why Top is drift-immune and contact is not.
    //
    // Deliberately observation-only. A hip-mounted board feels BOTH feet land -- the shock
    // crosses the pelvis -- so this says "a foot landed", never "which foot". The leg named in
    // the log is the ipsilateral assumption from boardToLeg, not a measurement.
    private void TrackFootContact(
        int board,
        imu_data_receive.ImuSampleSnapshot sample)
    {
        float magnitude = sample.Acceleration.magnitude;

        // Baseline is an EMA of |a|, so it absorbs gravity and posture without needing the board
        // orientation -- which is exactly the thing that is unreliable during walking.
        float dt = sample.HasDeltaTime ? sample.DeltaTime : 0.01f;
        float alpha = 1f - Mathf.Exp(-dt / Mathf.Max(0.02f, contactBaselineTau));

        if (!hasShockBaseline[board])
        {
            shockBaseline[board] = magnitude;
            hasShockBaseline[board] = true;
            return;
        }

        shockBaseline[board] += alpha * (magnitude - shockBaseline[board]);

        // Only positive excursions: a strike adds energy. Signed deviation would also fire on
        // the free-fall dip during swing.
        float shock = magnitude - shockBaseline[board];
        currentShock[board] = shock;

        if (shock > peakShockSinceReport[board])
        {
            peakShockSinceReport[board] = shock;
        }

        ReportShockPeaks();

        if (measureInternalLatency)
        {
            TrackShockPeak(board, shock, Time.realtimeSinceStartupAsDouble);
        }

        // The baseline and shock above are tracked unconditionally: they are the diagnostic used
        // to CHOOSE the threshold, so they have to be readable before the feature is switched on.
        // Only the firing below is gated.
        if (!detectFootContact)
        {
            return;
        }

        if (requireStartPress && !processingActive)
        {
            return;
        }

        double now = Time.realtimeSinceStartupAsDouble;

        // Two levels, not one: fire at the threshold, re-arm only well below it.
        if (shock < contactShockThreshold * contactReleaseFraction)
        {
            contactArmed[board] = true;
        }

        if (shock < contactShockThreshold)
        {
            return;
        }

        // Fire on the RISING edge. A strike stays above threshold for several samples, and the
        // onset is the physically meaningful instant -- the peak lags it by a few milliseconds
        // and wanders with surface stiffness.
        if (!contactArmed[board] || now < nextAllowedContactTime[board])
        {
            return;
        }

        contactArmed[board] = false;
        nextAllowedContactTime[board] = now + contactMinInterval;

        contactCount[board]++;
        lastContactShock[board] = shock;

        // One decision, three consumers. Attribution, the event and the beep used to keep
        // separate timers with the same window, so they could disagree about what counted as a
        // footfall -- and the beep is meant to be an audible readout of exactly what the rest
        // receives.
        if (!AcceptAsFootfall(now))
        {
            return;
        }

        AttributeFootfall(now, board);

        FootfallMeasured?.Invoke(new MeasuredFootfall(now, board, shock));

        if (measureInternalLatency)
        {
            RecordFootfallLatency(board, shock, now, sample.HostReceiveTime);
        }

        if (beepOnFootContact)
        {
            double beepStart = Time.realtimeSinceStartupAsDouble;

            PlayClip(GetContactTone(ResolveContactBeepFrequency()));

            if (measureInternalLatency)
            {
                lastBeepCallMs =
                    (float)((Time.realtimeSinceStartupAsDouble - beepStart) * 1000.0);
            }
        }

        if (logFootContact)
        {
            Debug.Log(
                $"[ImuGaitEventDetector] FootContact board={board} " +
                $"({ResolveLeg(board)} by assumption) shock={shock:F2} m/s^2 " +
                $"t={now:F3}.",
                this);
        }
    }

    // Reports all boards on one line so the two peaks can be compared directly -- the threshold
    // has to be chosen from the QUIETER board, and that is only obvious side by side.
    private void ReportShockPeaks()
    {
        // The window rolls whether or not anyone is logging it. Gating the RESET on logging
        // froze the Inspector diagnostic at an all-time high the moment logging was turned off,
        // which reads as "stuck" rather than "never cleared".
        double now = Time.realtimeSinceStartupAsDouble;

        if (now < nextShockReportTime)
        {
            return;
        }

        nextShockReportTime = now + Mathf.Max(0.25f, shockPeakReportInterval);

        string line = logShockPeaks ? "[ImuGaitEventDetector] Peak shock" : null;

        for (int board = 0; board < boardCount; board++)
        {
            if (line != null)
            {
                line +=
                    $" | board {board} ({ResolveLeg(board)}): {peakShockSinceReport[board]:F2}" +
                    $" (baseline {shockBaseline[board]:F2})";
            }

            peakShockSinceReport[board] = 0f;
            lastAttributedLeg = null;
            lastAttributionMargin = 0f;
        }

        if (line != null)
        {
            Debug.Log(line, this);
        }
    }

    /// <summary>
    /// Receives a leg verdict decided elsewhere. Kept as a push rather than the detector reading
    /// the attribution component, so the detector holds no reference to it and still runs
    /// unchanged if that component is removed.
    /// </summary>
    public void ReportExternalLegVerdict(PeakLegSide leg, bool confident)
    {
        externalLeg = leg;
        externalConfident = confident;
        externalVerdictTime = Time.realtimeSinceStartupAsDouble;
    }

    private float ResolveContactBeepFrequency()
    {
        if (!pitchBeepByLeg)
        {
            return contactBeepFrequency;
        }

        if (beepPitchSource == AttributionMethod.SwingDominance)
        {
            return ResolveSwingDominanceBeepFrequency();
        }

        if (!attributeContactLeg || !lastAttributedLeg.HasValue)
        {
            return contactBeepFrequency;
        }

        return lastAttributedLeg.Value == PeakLegSide.Left
            ? contactBeepFrequencyLeft
            : contactBeepFrequencyRight;
    }

    private float ResolveSwingDominanceBeepFrequency()
    {
        if (!externalLeg.HasValue)
        {
            return contactBeepFrequency;
        }

        // The verdict must belong to THIS footfall. Without the freshness check a footfall the
        // attribution component skipped would be announced with the previous step's leg, which
        // sounds authoritative and is wrong -- the worst kind of feedback.
        if (Time.realtimeSinceStartupAsDouble - externalVerdictTime > externalVerdictMaxAgeSeconds)
        {
            return contactBeepFrequency;
        }

        // An ambiguous verdict gets the neutral tone rather than a guess, so a pitched beep
        // always means "confidently this leg" and never "probably-ish".
        if (requireConfidentVerdictForPitch && !externalConfident)
        {
            return contactBeepFrequency;
        }

        return externalLeg.Value == PeakLegSide.Left
            ? contactBeepFrequencyLeft
            : contactBeepFrequencyRight;
    }

    // Cached per frequency: contact beeps fire ~2x a second for a whole session, and a fresh
    // float[] per footfall is avoidable GC pressure in VR.
    private AudioClip GetContactTone(float frequency)
    {
        int key = Mathf.RoundToInt(frequency) * 1000 + Mathf.RoundToInt(contactBeepDuration * 1000f);

        AudioClip clip;

        if (!contactToneCache.TryGetValue(key, out clip) || clip == null)
        {
            clip = CreateTone(frequency, contactBeepDuration);
            contactToneCache[key] = clip;
        }

        return clip;
    }

    // Starting to walk from standing produces TWO shocks close together -- the weight shift off
    // the standing leg, then the first foot landing -- roughly half a step apart. Mid-walk those
    // merge into the rhythm and the ordinary window absorbs them, so widening that window for
    // every step would cost real steps to fix a first-step artefact. Instead the step that
    // follows a bout start gets its own, longer window.
    private bool AcceptAsFootfall(double now)
    {
        double gap = now - lastFootfallRaisedTime;

        float required = contactBeepDedupeWindow;

        if (adaptiveFootfallSpacing && meanFootfallInterval > 0f)
        {
            // The floor keeps the fixed window as a backstop, so a cadence estimate that has
            // drifted low can never open the gate wider than the operator asked for.
            required = Mathf.Max(
                required,
                meanFootfallInterval * adaptiveSpacingFraction);
        }

        if (gap < required)
        {
            consecutiveSpacingRejections++;

            if (consecutiveSpacingRejections < spacingRejectionsBeforeReset)
            {
                return false;
            }

            // Escape hatch: repeated rejections mean the estimate is wrong for this wearer or
            // pace, not that these are all duplicates. Accept and relearn from here.
            meanFootfallInterval = 0f;
        }

        consecutiveSpacingRejections = 0;

        // Only plausible step intervals train the estimate. A pause between bouts is not a
        // stride, and letting one in would inflate the mean and start rejecting real steps.
        if (gap > 0.2 && gap < 2.0)
        {
            meanFootfallInterval = meanFootfallInterval <= 0f
                ? (float)gap
                : Mathf.Lerp(meanFootfallInterval, (float)gap, 0.25f);
        }

        lastFootfallRaisedTime = now;

        return true;
    }

    // Reports the SOURCE signal, not anything derived from it. Every threshold, event and angle
    // in this component is a function of the swing rate, so when derived numbers stop making
    // sense this is the thing to look at first.
    private void ReportSwingRateRange()
    {
        if (!logSwingRateRange)
        {
            return;
        }

        double now = Time.realtimeSinceStartupAsDouble;

        if (now < nextSwingRateReportTime)
        {
            return;
        }

        nextSwingRateReportTime = now + Mathf.Max(0.25f, swingRateReportInterval);

        string line = "[ImuGaitEventDetector] Swing signal";

        for (int board = 0; board < boardCount; board++)
        {
            line +=
                $" | b{board}({ResolveLeg(board)}) rate {rateMinSinceReport[board]:F2}..{rateMaxSinceReport[board]:F2} rad/s," +
                $" angle {angleMinSinceReport[board]:F0}..{angleMaxSinceReport[board]:F0} deg," +
                $" | b{board} raw" +
                $" X {axisMinSinceReport[board, 0]:F2}..{axisMaxSinceReport[board, 0]:F2}" +
                $" Y {axisMinSinceReport[board, 1]:F2}..{axisMaxSinceReport[board, 1]:F2}" +
                $" Z {axisMinSinceReport[board, 2]:F2}..{axisMaxSinceReport[board, 2]:F2}," +
                $" tops={topsSinceReport[board]}," +
                $" thr={ResolveMinSwingRate(board):F2}," +
                $" axis={boardGyroAxis[board]}, inv={boardInvert[board]}";

            rateMinSinceReport[board] = 0f;
            rateMaxSinceReport[board] = 0f;
            angleMinSinceReport[board] = 0f;
            angleMaxSinceReport[board] = 0f;
            topsSinceReport[board] = 0;

            for (int axis = 0; axis < 3; axis++)
            {
                axisMinSinceReport[board, axis] = 0f;
                axisMaxSinceReport[board, axis] = 0f;
            }
        }

        Debug.Log(line, this);
    }

    // ===== Leg attribution =====
    private void IntegrateThighAngle(
        int board,
        float rate,
        imu_data_receive.ImuSampleSnapshot sample)
    {
        float dt = sample.HasDeltaTime ? sample.DeltaTime : 0.01f;

        thighAngleRad[board] += rate * dt;

        thighAngleSinceTop[board] = thighAngleRad[board] * Mathf.Rad2Deg;

        Vector3 corrected = sample.AngularVelocity - gyroBias[board];

        for (int axis = 0; axis < 3; axis++)
        {
            float value = corrected[axis];

            if (value < axisMinSinceReport[board, axis]) axisMinSinceReport[board, axis] = value;
            if (value > axisMaxSinceReport[board, axis]) axisMaxSinceReport[board, axis] = value;
        }

        float rateNow = currentSwingRate[board];
        float angleNow = thighAngleSinceTop[board];

        if (rateNow < rateMinSinceReport[board]) rateMinSinceReport[board] = rateNow;
        if (rateNow > rateMaxSinceReport[board]) rateMaxSinceReport[board] = rateNow;
        if (angleNow < angleMinSinceReport[board]) angleMinSinceReport[board] = angleNow;
        if (angleNow > angleMaxSinceReport[board]) angleMaxSinceReport[board] = angleNow;

        ReportSwingRateRange();
        timeSinceTop[board] =
            (float)(Time.realtimeSinceStartupAsDouble - lastTopTime[board]);
    }

    // Called once per FOOTFALL, not once per contact: both boards feel every step, so evaluating
    // per contact would score the same footfall twice and inflate the agreement rate.
    private void AttributeFootfall(double now, int firstBoard)
    {
        if (!attributeContactLeg || boardCount < 2)
        {
            return;
        }

        if (now - lastAttributionTime < contactBeepDedupeWindow)
        {
            return;
        }

        lastAttributionTime = now;

        // Method A -- thigh position. At heel strike the landing thigh is still well forward
        // while the trailing thigh has swung far back, so the board whose angle has travelled
        // LEAST since its own Top is the one landing. Roughly 45 degrees of separation.
        int angleBoard = 0;
        int timingBoard = 0;

        for (int board = 1; board < boardCount; board++)
        {
            if (thighAngleRad[board] > thighAngleRad[angleBoard])
            {
                angleBoard = board;
            }

            // Method B -- elapsed time. The landing leg's swing ended ~0.25 s ago, the stance
            // leg's ~0.8 s ago. Same ordering as A, but derived from time rather than position,
            // so the two fail in different ways.
            if (lastTopTime[board] > lastTopTime[timingBoard])
            {
                timingBoard = board;
            }
        }

        PeakLegSide angleLeg = ResolveLeg(angleBoard);
        PeakLegSide timingLeg = ResolveLeg(timingBoard);

        lastAttributionByAngle = angleLeg.ToString();
        lastAttributionByTiming = timingLeg.ToString();

        lastAttributedLeg =
            beepPitchSource == AttributionMethod.Angle ? angleLeg : timingLeg;

        // Confidence comes from the SEPARATION, not from the two methods agreeing: they share a
        // signal, so agreement says they are consistent, never that they are right. A stride
        // where both legs peaked at nearly the same time is genuinely undecidable, and saying so
        // is more useful than emitting a coin-flip verdict.
        float margin = Mathf.Abs(timeSinceTop[0] - timeSinceTop[1]);
        lastAttributionMargin = margin;

        bool confident = margin >= attributionMinMargin;

        if (confident)
        {
            attributionConfidentCount++;
        }
        else
        {
            attributionAmbiguousCount++;
        }

        bool agree = angleBoard == timingBoard;

        if (agree)
        {
            attributionAgreeCount++;
        }
        else
        {
            attributionDisagreeCount++;
        }

        if (logLegAttribution)
        {
            Debug.Log(
                $"[ImuGaitEventDetector] Footfall t={now:F3} firstBoard={firstBoard} " +
                $"| angle={angleLeg} ({thighAngleSinceTop[0]:F1} vs {thighAngleSinceTop[1]:F1} deg) " +
                $"| timing={timingLeg} ({timeSinceTop[0]:F2} vs {timeSinceTop[1]:F2} s) " +
                $"| margin={margin:F2}s {(confident ? "CONFIDENT" : "AMBIGUOUS")} " +
                $"| {(agree ? "AGREE" : "DISAGREE")}",
                this);
        }
    }

    // Slow high-pass on the swing rate. Deliberately much longer than a stride: over whole gait
    // cycles the thigh returns to where it started, so the true mean rate is zero and anything
    // left over that window is bias.
    private float RemoveRateDrift(
        int board,
        float rate,
        imu_data_receive.ImuSampleSnapshot sample)
    {
        if (!removeRateDrift)
        {
            return rate;
        }

        if (!hasRateDrift[board])
        {
            rateDrift[board] = rate;
            hasRateDrift[board] = true;
            return 0f;
        }

        float dt = sample.HasDeltaTime ? sample.DeltaTime : 0.01f;
        float alpha = 1f - Mathf.Exp(-dt / Mathf.Max(0.5f, rateDriftTau));

        rateDrift[board] += alpha * (rate - rateDrift[board]);

        return rate - rateDrift[board];
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

        // One gate for every event type, placed here rather than at the sample handler so the
        // drift filter, the shock baseline and the comparison history all stay warm while the
        // wearer is getting ready.
        if (requireStartPress && !processingActive)
        {
            return;
        }

        PeakLegSide leg = ResolveLeg(board);

        if (eventType == JasmineLegEventType.Top)
        {
            float selfPeak;
            float otherPeak;
            CrossBoardPeaks(board, now, out selfPeak, out otherPeak);

            float ratio = otherPeak > 0.0001f ? selfPeak / otherPeak : float.PositiveInfinity;
            lastCrossBoardRatio = ratio;

            bool floorOk = selfPeak >= crossBoardFloor;
            bool ratioOk = ratio >= crossBoardMinRatio;
            bool amplitudeOk = swingAmplitude >= ResolveMinSwingRate(board);

            // Both verdicts are logged on every candidate, whichever gate is in use, so the two
            // can be compared on the same walk before either is trusted.
            if (logCrossBoardRatios)
            {
                string floorText = floorOk ? "ok" : "LOW";
                string ratioText = ratioOk ? "pass" : "fail";
                string amplitudeText = amplitudeOk ? "pass" : "fail";
                string inUse = useCrossBoardGate ? "ratio" : "amplitude";

                double sameBoardGap = lastCrossingTime[board] > 0.0
                    ? now - lastCrossingTime[board]
                    : -1.0;

                double anyBoardGap = lastAnyCrossingTime > 0.0
                    ? now - lastAnyCrossingTime
                    : -1.0;

                Debug.Log(
                    $"[CrossBoard] t={now:F3} {leg} " +
                    $"dtSame={sameBoardGap:F3} dtAny={anyBoardGap:F3} " +
                    $"| self={selfPeak:F2} other={otherPeak:F2} ratio={ratio:F2} " +
                    $"floor={floorText} ratioGate={ratioText} " +
                    $"| amplitude={swingAmplitude:F2} vs thr={ResolveMinSwingRate(board):F2} " +
                    $"{amplitudeText} | in use: {inUse}",
                    this);
            }

            lastCrossingTime[board] = now;
            lastAnyCrossingTime = now;

            if (useCrossBoardGate)
            {
                if (!floorOk)
                {
                    suppressedByAmplitude++;
                    return;
                }

                if (!ratioOk)
                {
                    suppressedByCrossBoard++;
                    return;
                }
            }
            else if (!amplitudeOk)
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

    private float ResolveMinSwingRate(int board)
    {
        if (minSwingRatePerBoard != null &&
            board < minSwingRatePerBoard.Length &&
            minSwingRatePerBoard[board] > 0f)
        {
            return minSwingRatePerBoard[board];
        }

        return minSwingRate;
    }

    /// <summary>Board-to-leg mapping, exposed so other components share one source of truth
    /// rather than duplicating the array and drifting out of step with it.</summary>
    public PeakLegSide ResolveLegForBoard(int board)
    {
        return ResolveLeg(board);
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

    private void Update()
    {
        PollCalibrationButton();
        PollProcessingGate();
    }

    // Toggled rather than held: a run lasts as long as the walk, and holding a controller button
    // through it would change how the wearer swings that arm.
    private void PollProcessingGate()
    {
        if (!requireStartPress || calibrationRunning || !Application.isPlaying)
        {
            return;
        }

        double now = Time.realtimeSinceStartupAsDouble;

        if (now - lastStartPressTime < startPressLockout)
        {
            return;
        }

        bool pressed = Input.GetKeyDown(startKey);

        if (!pressed && allowControllerStart)
        {
            try
            {
                pressed = OVRInput.GetDown(startButton, startController);
            }
            catch (System.Exception)
            {
                // No OVR runtime here; the keyboard path still works.
            }
        }

        if (!pressed)
        {
            return;
        }

        lastStartPressTime = now;
        processingActive = !processingActive;

        if (processingActive && resetCountersOnStart)
        {
            ResetDiagnosticCounters();
        }

        Debug.Log(
            $"[ImuGaitEventDetector] Processing {(processingActive ? "STARTED" : "STOPPED")}" +
            $"{(processingActive && resetCountersOnStart ? " (counters zeroed)" : "")}.",
            this);
    }

    /// <summary>
    /// Zeroes everything a run is measured by. Deliberately does NOT touch calibration, the
    /// swing axis, thresholds or the filter states -- those are what make the next run
    /// comparable to the last one.
    /// </summary>
    private void ResetDiagnosticCounters()
    {
        leftTopCount = 0;
        rightTopCount = 0;
        suppressedByRefractory = 0;
        suppressedByAlternation = 0;
        suppressedByAmplitude = 0;
        suppressedByCrossBoard = 0;
        lastAnyCrossingTime = 0.0;

        for (int board = 0; board < boardCount; board++)
        {
            if (contactCount != null && board < contactCount.Length)
            {
                contactCount[board] = 0;
            }

            if (lastCrossingTime != null && board < lastCrossingTime.Length)
            {
                lastCrossingTime[board] = 0.0;
            }
        }

        lastSubmittedLeg = null;
    }

    // Hold-to-start, with the progress surfaced in the Inspector and a rising tick each half
    // second so the wearer knows it is counting without seeing a screen.
    private void PollCalibrationButton()
    {
        if (calibrationRunning || !Application.isPlaying)
        {
            calibrateHeldFor = 0f;
            return;
        }

        bool held = false;

        if (allowControllerCalibration)
        {
            held = OVRInput.Get(calibrateButton, calibrateController);

            // Reported unconditionally so a dead controller is distinguishable from a wrong
            // button: "no controller" is a hardware/rig problem, "connected but not down" is not.
            OVRInput.Controller active = OVRInput.GetActiveController();

            calibrateInputDebug =
                active == OVRInput.Controller.None
                    ? "no controller"
                    : $"active={active} target={calibrateButton} down={held} " +
                      $"anyButton={OVRInput.Get(OVRInput.RawButton.Any)}";
        }
        else
        {
            calibrateInputDebug = "controller input disabled";
        }

        if (allowKeyboardCalibration && Input.GetKey(calibrateKey))
        {
            held = true;
        }

        if (!held)
        {
            // Logged because the wearer cannot read the Inspector: a hold that keeps aborting
            // partway looks identical, from inside the headset, to one that never registered.
            if (calibrateHeldFor > 0f)
            {
                Debug.Log(
                    $"[ImuGaitEventDetector] Calibration hold released after " +
                    $"{calibrateHeldFor:F2}s of {calibrateHoldSeconds:F2}s -- not started. " +
                    $"Input: {calibrateInputDebug}",
                    this);

                calibrateHoldStatus = "";
            }

            calibrateHeldFor = 0f;
            return;
        }

        float previous = calibrateHeldFor;
        calibrateHeldFor += Time.unscaledDeltaTime;

        calibrateHoldStatus =
            $"hold {calibrateHeldFor:F1}/{calibrateHoldSeconds:F1}s";

        // One tick per half second of hold, climbing in pitch, so the countdown is audible.
        if (Mathf.FloorToInt(calibrateHeldFor / 0.5f) > Mathf.FloorToInt(previous / 0.5f))
        {
            PlayTone(600f + 200f * Mathf.FloorToInt(calibrateHeldFor / 0.5f), 0.05f);
        }

        if (calibrateHeldFor >= calibrateHoldSeconds)
        {
            calibrateHeldFor = 0f;
            calibrateHoldStatus = "";

            Debug.Log(
                $"[ImuGaitEventDetector] Calibration started from {calibrateController} " +
                $"{calibrateButton}.",
                this);

            CalibrateDetector();
        }
    }

    // ===== Calibration audio =====
    // The tone is synthesised rather than referenced as a clip, so calibration stays a
    // single-component drop-in with no asset to wire up or lose.
    private void PlayBeep(float frequency)
    {
        if (!calibrationBeeps)
        {
            return;
        }

        PlayTone(frequency, beepDuration);
    }

    private void PlayTone(float frequency, float duration)
    {
        PlayClip(CreateTone(frequency, duration));
    }

    private void PlayClip(AudioClip clip)
    {
        if (clip == null || beepVolume <= 0f)
        {
            return;
        }

        if (beepSource == null)
        {
            beepSource = gameObject.AddComponent<AudioSource>();
            beepSource.playOnAwake = false;
            beepSource.spatialBlend = 0f; // 2D: the cue is an instruction, not a world sound.
        }

        beepSource.PlayOneShot(clip, beepVolume);
    }

    private static AudioClip CreateTone(float frequency, float duration)
    {
        const int sampleRate = 44100;

        int sampleCount = Mathf.Max(1, Mathf.RoundToInt(sampleRate * duration));
        float[] data = new float[sampleCount];

        // A short raised-cosine fade at each end. A bare sine switched on and off mid-cycle
        // produces an audible click that is easy to mistake for the beep itself.
        int fade = Mathf.Max(1, sampleCount / 20);

        for (int i = 0; i < sampleCount; i++)
        {
            float envelope = 1f;

            if (i < fade)
            {
                envelope = i / (float)fade;
            }
            else if (i >= sampleCount - fade)
            {
                envelope = (sampleCount - 1 - i) / (float)fade;
            }

            data[i] =
                Mathf.Sin(2f * Mathf.PI * frequency * i / sampleRate) * envelope;
        }

        AudioClip clip = AudioClip.Create(
            $"CalibrationBeep{Mathf.RoundToInt(frequency)}",
            sampleCount,
            1,
            sampleRate,
            false);

        clip.SetData(data, 0);

        return clip;
    }

    // ===== Calibration persistence =====
    // Runtime writes to serialized fields are rolled back when Play ends, so calibration results
    // do not survive the session that produced them. Stored outside the scene instead.
    private void SaveCalibration()
    {
        if (!persistCalibration)
        {
            return;
        }

        PlayerPrefs.SetInt($"{calibrationKey}_boards", boardCount);
        PlayerPrefs.SetFloat($"{calibrationKey}_fallback", minSwingRate);

        for (int board = 0; board < boardCount; board++)
        {
            string k = $"{calibrationKey}_b{board}";

            PlayerPrefs.SetFloat($"{k}_thr", minSwingRatePerBoard[board]);
            PlayerPrefs.SetInt($"{k}_axis", (int)boardGyroAxis[board]);
            PlayerPrefs.SetInt($"{k}_inv", boardInvert[board] ? 1 : 0);
            PlayerPrefs.SetFloat($"{k}_sx", boardSwingAxis[board].x);
            PlayerPrefs.SetFloat($"{k}_sy", boardSwingAxis[board].y);
            PlayerPrefs.SetFloat($"{k}_sz", boardSwingAxis[board].z);
            PlayerPrefs.SetFloat($"{k}_bx", gyroBias[board].x);
            PlayerPrefs.SetFloat($"{k}_by", gyroBias[board].y);
            PlayerPrefs.SetFloat($"{k}_bz", gyroBias[board].z);
        }

        PlayerPrefs.Save();

        Debug.Log(
            $"[ImuGaitEventDetector] Calibration saved under \"{calibrationKey}\".",
            this);
    }

    private void LoadCalibration()
    {
        if (!persistCalibration ||
            !PlayerPrefs.HasKey($"{calibrationKey}_boards"))
        {
            return;
        }

        int saved = PlayerPrefs.GetInt($"{calibrationKey}_boards");

        EnsureBoardState(saved);

        minSwingRate = PlayerPrefs.GetFloat($"{calibrationKey}_fallback", minSwingRate);

        for (int board = 0; board < saved && board < boardCount; board++)
        {
            string k = $"{calibrationKey}_b{board}";

            minSwingRatePerBoard[board] = PlayerPrefs.GetFloat($"{k}_thr", 0f);
            boardGyroAxis[board] = (GyroAxis)PlayerPrefs.GetInt($"{k}_axis", 0);
            boardInvert[board] = PlayerPrefs.GetInt($"{k}_inv", 0) != 0;

            boardSwingAxis[board] = new Vector3(
                PlayerPrefs.GetFloat($"{k}_sx", 0f),
                PlayerPrefs.GetFloat($"{k}_sy", 0f),
                PlayerPrefs.GetFloat($"{k}_sz", 0f));

            gyroBias[board] = new Vector3(
                PlayerPrefs.GetFloat($"{k}_bx", 0f),
                PlayerPrefs.GetFloat($"{k}_by", 0f),
                PlayerPrefs.GetFloat($"{k}_bz", 0f));
        }

        calibrated = true;
        calibrationStage = "loaded from saved";

        string thresholds = "";

        for (int board = 0; board < boardCount; board++)
        {
            thresholds += $" b{board}={minSwingRatePerBoard[board]:F3};";
        }

        Debug.Log(
            $"[ImuGaitEventDetector] Calibration loaded from \"{calibrationKey}\":{thresholds}",
            this);
    }

    [ContextMenu("Clear Saved Calibration")]
    public void ClearSavedCalibration()
    {
        PlayerPrefs.DeleteKey($"{calibrationKey}_boards");
        PlayerPrefs.Save();

        Debug.Log(
            $"[ImuGaitEventDetector] Saved calibration cleared for \"{calibrationKey}\".",
            this);
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
        calibrationOuter = new float[boardCount, 3, 3];
        calibrationAxisMax = new float[boardCount, 3];
        calibrationAxisTop = new float[boardCount, 3, Mathf.Max(1, calibrationPeakRank)];
        calibrationWalkSamples = new int[boardCount];

        calibrationPhase = 1;
        calibrationStage = "Phase 1/2: STAND STILL";
        Debug.Log(
            $"[ImuGaitEventDetector] Calibration phase 1: stand still for " +
            $"{stillCalibrationSeconds:F1}s.",
            this);

        PlayBeep(beepStartFrequency);

        yield return new WaitForSecondsRealtime(stillCalibrationSeconds);

        PlayBeep(beepStartFrequency * 0.5f);

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

        if (transitionSeconds > 0f)
        {
            calibrationPhase = 3;
            calibrationStage = $"Transition: get ready to walk ({transitionSeconds:F0}s)";
            Debug.Log(
                $"[ImuGaitEventDetector] Transition: {transitionSeconds:F1}s to start walking. " +
                "Nothing is recorded during this window.",
                this);

            yield return new WaitForSecondsRealtime(transitionSeconds);
        }

        calibrationPhase = 2;
        calibrationStage = "Phase 2/2: WALK NORMALLY";
        Debug.Log(
            $"[ImuGaitEventDetector] Calibration phase 2: walk normally for " +
            $"{walkCalibrationSeconds:F1}s.",
            this);

        PlayBeep(beepStartFrequency);

        // The bias is now known, so the walk phase accumulates bias-corrected statistics.
        yield return new WaitForSecondsRealtime(walkCalibrationSeconds);

        PlayBeep(beepStartFrequency * 0.5f);

        ResolveCalibrationResults();

        calibrationRunning = false;
        calibrationPhase = 0;
        calibrationBiasSum = null;
        calibrationBiasSamples = null;
        calibrationAxisSum = null;
        calibrationAxisSumSq = null;
        calibrationAxisSumCube = null;
        calibrationOuter = null;
        calibrationAxisMax = null;
        calibrationAxisTop = null;
        calibrationWalkSamples = null;

        ResetRunningState();
        SaveCalibration();
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

        // Phase 3 is the hand-over window. Those samples are neither still nor steady walking,
        // so folding them into either statistic would bias it -- discard them outright.
        if (calibrationPhase != 1 && calibrationPhase != 2)
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

            for (int other = 0; other < 3; other++)
            {
                calibrationOuter[board, axis, other] += value * corrected[other];
            }

            float magnitude = Mathf.Abs(value);

            if (magnitude > calibrationAxisMax[board, axis])
            {
                calibrationAxisMax[board, axis] = magnitude;
            }

            InsertCalibrationPeak(board, axis, magnitude);
        }

        calibrationWalkSamples[board]++;
    }

    // Largest eigenvector of the gyro covariance, by power iteration. That direction is the one
    // the thigh actually rotates about; projecting onto it recovers the whole swing instead of
    // whatever fraction happens to land on X, Y or Z.
    private Vector3 SolvePrincipalAxis(
        int board,
        int samples,
        int bestAxis,
        bool invert)
    {
        float[,] cov = new float[3, 3];

        for (int r = 0; r < 3; r++)
        {
            float meanR = calibrationAxisSum[board, r] / samples;

            for (int c = 0; c < 3; c++)
            {
                float meanC = calibrationAxisSum[board, c] / samples;

                cov[r, c] = calibrationOuter[board, r, c] / samples - meanR * meanC;
            }
        }

        // Seeded with the best single axis, which is already close to the answer, so a handful
        // of iterations converge.
        Vector3 v = Vector3.zero;
        v[bestAxis] = 1f;

        for (int iteration = 0; iteration < 64; iteration++)
        {
            Vector3 next = new Vector3(
                cov[0, 0] * v.x + cov[0, 1] * v.y + cov[0, 2] * v.z,
                cov[1, 0] * v.x + cov[1, 1] * v.y + cov[1, 2] * v.z,
                cov[2, 0] * v.x + cov[2, 1] * v.y + cov[2, 2] * v.z);

            if (next.sqrMagnitude < 1e-12f)
            {
                break;
            }

            v = next.normalized;
        }

        // Eigenvectors have arbitrary sign. Orient it to agree with the skew-derived direction
        // of the dominant single axis, which is the only sign information calibration has.
        float reference = invert ? -1f : 1f;

        if (v[bestAxis] * reference < 0f)
        {
            v = -v;
        }

        return v;
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

            boardSwingAxis[board] =
                SolvePrincipalAxis(board, samples, bestAxis, skewness < 0f);

            float peakRate = RobustCalibrationPeak(board, bestAxis);
            float rawPeak = calibrationAxisMax[board, bestAxis];
            globalPeakRate = Mathf.Max(globalPeakRate, peakRate);
            anyBoardResolved = true;

            // Each board is thresholded against its OWN peak. Pocket tightness and how squarely
            // the board sits differ between hips, so the two peaks legitimately differ; taking
            // the maximum and applying it to both makes the quieter board miss real steps.
            minSwingRatePerBoard[board] = Mathf.Max(0.05f, peakRate * thresholdFraction);

            Debug.Log(
                $"[ImuGaitEventDetector] Board {board} ({ResolveLeg(board)}): " +
                $"axis={boardGyroAxis[board]}, invert={boardInvert[board]}, " +
                $"swingAxis={boardSwingAxis[board]}, " +
                $"peak={peakRate:F3} rad/s (raw max {rawPeak:F3}, rank " +
                $"{calibrationPeakRank}), sd={standardDeviation:F3}, " +
                $"skew={skewness:F3}, samples={samples}.",
                this);
        }

        if (anyBoardResolved && globalPeakRate > 0f)
        {
            // Kept only as the fallback for a board calibration could not resolve.
            minSwingRate = Mathf.Max(0.05f, globalPeakRate * thresholdFraction);
            calibrated = true;
            calibrationStage = "done";

            string thresholds = "";

            for (int board = 0; board < boardCount; board++)
            {
                thresholds +=
                    $" board {board} ({ResolveLeg(board)})={minSwingRatePerBoard[board]:F3};";
            }

            Debug.Log(
                $"[ImuGaitEventDetector] Calibration complete. Per-board minSwingRate:{thresholds} " +
                $"fallback={minSwingRate:F3} rad/s. " +
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
        Resize(ref shockBaseline, newCount);
        Resize(ref hasShockBaseline, newCount);
        Resize(ref contactArmed, newCount);
        Resize(ref nextAllowedContactTime, newCount);
        Resize(ref contactCount, newCount);
        Resize(ref lastContactShock, newCount);
        Resize(ref currentShock, newCount);
        Resize(ref peakShockSinceReport, newCount);
        Resize(ref latencyWatching, newCount);
        Resize(ref latencyFireTime, newCount);
        Resize(ref latencyPeakShock, newCount);
        Resize(ref latencyPeakTime, newCount);
        Resize(ref thighAngleRad, newCount);
        Resize(ref lastTopTime, newCount);
        Resize(ref lastTopResetTime, newCount);
        Resize(ref thighAngleSinceTop, newCount);
        Resize(ref timeSinceTop, newCount);
        Resize(ref minSwingRatePerBoard, newCount);
        Resize(ref crossMagnitude, newCount);
        Resize(ref crossTime, newCount);
        Resize(ref crossWrite, newCount);
        Resize(ref lastCrossingTime, newCount);
        Resize(ref swingRising, newCount);
        Resize(ref hasSwingState, newCount);
        Resize(ref crossCount, newCount);
        Resize(ref rateMinSinceReport, newCount);
        Resize(ref rateMaxSinceReport, newCount);
        Resize(ref angleMinSinceReport, newCount);
        Resize(ref angleMaxSinceReport, newCount);
        Resize(ref topsSinceReport, newCount);
        Resize(ref rateDrift, newCount);
        Resize(ref boardSwingAxis, newCount);
        Resize(ref hasRateDrift, newCount);

        axisMinSinceReport = new float[newCount, 3];
        axisMaxSinceReport = new float[newCount, 3];

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

    // Latency probe state. Per board, because two boards can be inside a shock at once and a
    // single slot would let one board's peak be credited to the other.
    private bool[] latencyWatching;
    private double[] latencyFireTime;
    private float[] latencyPeakShock;
    private double[] latencyPeakTime;

    private double queueSumMs;
    private double peakLeadSumMs;
    private int peakLeadCount;

    // Called on every sample while a shock is still above the re-arm level, so the true peak is
    // found by watching rather than guessed from the firing sample.
    private void TrackShockPeak(int board, float shock, double now)
    {
        if (!latencyWatching[board])
        {
            return;
        }

        if (shock > latencyPeakShock[board])
        {
            latencyPeakShock[board] = shock;
            latencyPeakTime[board] = now;
        }

        // The shock has rung down; the peak is settled.
        if (shock < contactShockThreshold * contactReleaseFraction)
        {
            latencyWatching[board] = false;

            lastPeakLeadMs = (float)((latencyPeakTime[board] - latencyFireTime[board]) * 1000.0);
            peakLeadSumMs += lastPeakLeadMs;
            peakLeadCount++;
            meanPeakLeadMs = (float)(peakLeadSumMs / peakLeadCount);
        }
    }

    private void RecordFootfallLatency(
        int board,
        float shock,
        double now,
        double hostReceiveTime)
    {
        // A host receive time of zero means the sample predates this instrumentation or the
        // clock offset has not been established yet; scoring it would drag the mean to nonsense.
        if (hostReceiveTime <= 0.0)
        {
            return;
        }

        lastQueueMs = (float)((now - hostReceiveTime) * 1000.0);
        queueSumMs += lastQueueMs;
        latencySamples++;
        meanQueueMs = (float)(queueSumMs / latencySamples);

        if (lastQueueMs > worstQueueMs)
        {
            worstQueueMs = lastQueueMs;
        }

        latencyWatching[board] = true;
        latencyFireTime[board] = now;
        latencyPeakShock[board] = shock;
        latencyPeakTime[board] = now;

        if (latencySamples % Mathf.Max(1, latencyReportEvery) == 0)
        {
            Debug.Log(
                $"[LatencyProbe] n={latencySamples} " +
                $"| queue mean={meanQueueMs:F1} ms last={lastQueueMs:F1} worst={worstQueueMs:F1} " +
                $"| peak lead mean={meanPeakLeadMs:F1} ms " +
                $"| beep call={lastBeepCallMs:F2} ms " +
                "| physical detection lag NOT measured here.",
                this);
        }
    }

    // Signed projected swing-rate history per board, for the cross-board comparison.
    //
    // Raw |gyro| magnitude was tried first and measured useless here: at peak flexion of one
    // thigh the other is in mid-stance, rotating at a similar SPEED in the opposite DIRECTION,
    // so magnitudes tied at ~1.0 while the projection on the same steps separated 2:1. Magnitude
    // discards the sign, and the sign is the entire difference between a swinging leg and a
    // stance leg. Only positive (flexion) peaks are compared, so an extending thigh cannot win.
    private float[][] crossMagnitude;
    private double[][] crossTime;
    private int[] crossWrite;

    // When each board last produced a crossing, accepted or not. Without this the log cannot
    // distinguish one swing chopped into fragments from two genuine swings, which are the same
    // two lines on the page and completely different faults.
    private double[] lastCrossingTime;

    // Which side of the dead zone each board is currently committed to. This is the memory that
    // a bare sign test lacks: without it, a signal resting inside the band has no defined state
    // and every jitter looks like a transition.
    private bool[] swingRising;
    private bool[] hasSwingState;
    private int[] crossCount;

    private const int CrossHistoryLength = 256;

    private double lastAnyCrossingTime;

    private double lastStartPressTime = -10.0;

    private void TrackCrossBoardRate(int board, float projectedRate)
    {
        if (crossMagnitude == null || board >= crossMagnitude.Length)
        {
            return;
        }

        if (crossMagnitude[board] == null)
        {
            crossMagnitude[board] = new float[CrossHistoryLength];
            crossTime[board] = new double[CrossHistoryLength];
        }

        int w = crossWrite[board];
        crossMagnitude[board][w] = projectedRate;
        crossTime[board][w] = Time.realtimeSinceStartupAsDouble;

        crossWrite[board] = (w + 1) % CrossHistoryLength;

        if (crossCount[board] < CrossHistoryLength)
        {
            crossCount[board]++;
        }
    }

    private float CrossWindowPeak(int board, double now)
    {
        if (crossMagnitude == null ||
            board >= crossMagnitude.Length ||
            crossMagnitude[board] == null)
        {
            return 0f;
        }

        double cutoff = now - crossBoardWindowSeconds;
        float peak = 0f;

        for (int i = 0; i < crossCount[board]; i++)
        {
            int index = (crossWrite[board] - 1 - i + CrossHistoryLength) % CrossHistoryLength;

            // History is walked newest-first, so the first sample older than the window ends it.
            if (crossTime[board][index] < cutoff)
            {
                break;
            }

            // peak starts at zero, so this is the largest POSITIVE rate in the window. A board
            // that only extended during the window scores zero, which is the correct answer.
            if (crossMagnitude[board][index] > peak)
            {
                peak = crossMagnitude[board][index];
            }
        }

        return peak;
    }

    // Compares against the LOUDEST other board rather than a named one, so a third board could
    // be added without this logic changing meaning.
    private void CrossBoardPeaks(int board, double now, out float self, out float other)
    {
        self = CrossWindowPeak(board, now);
        other = 0f;

        for (int b = 0; b < boardCount; b++)
        {
            if (b == board)
            {
                continue;
            }

            float peak = CrossWindowPeak(b, now);

            if (peak > other)
            {
                other = peak;
            }
        }
    }

    // Insertion sort into a short descending list. The list is tiny (about 20), so this costs
    // less than keeping a heap and stays readable.
    private void InsertCalibrationPeak(int board, int axis, float magnitude)
    {
        if (calibrationAxisTop == null)
        {
            return;
        }

        int rank = calibrationAxisTop.GetLength(2);

        if (magnitude <= calibrationAxisTop[board, axis, rank - 1])
        {
            return;
        }

        int slot = rank - 1;

        while (slot > 0 && calibrationAxisTop[board, axis, slot - 1] < magnitude)
        {
            calibrationAxisTop[board, axis, slot] = calibrationAxisTop[board, axis, slot - 1];
            slot--;
        }

        calibrationAxisTop[board, axis, slot] = magnitude;
    }

    /// <summary>
    /// The calibration peak, read as the Nth largest sample rather than the largest.
    ///
    /// A single knock against the board produces a sample far above anything walking does, and
    /// the threshold derived from it can sit above every real swing -- which silences that board
    /// for the whole session, with nothing to indicate why. Reading down the ranking discards
    /// short spikes while genuine peaks, which recur on every stride, are unaffected.
    /// </summary>
    private float RobustCalibrationPeak(int board, int axis)
    {
        if (calibrationAxisTop == null)
        {
            return calibrationAxisMax[board, axis];
        }

        int rank = calibrationAxisTop.GetLength(2);
        float value = calibrationAxisTop[board, axis, rank - 1];

        // Too few samples to fill the ranking -- a very short walk phase. Fall back to the
        // largest seen rather than to the zero that would otherwise be sitting in the last slot.
        if (value <= 0f)
        {
            return calibrationAxisMax[board, axis];
        }

        return value;
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

            // The baseline is re-seeded rather than zeroed: a zero baseline would read the first
            // sample's ~9.81 as a huge shock and fire a phantom contact.
            hasRateDrift[board] = false;
            hasShockBaseline[board] = false;
            contactArmed[board] = false;
            nextAllowedContactTime[board] = 0.0;
            currentShock[board] = 0f;
            peakShockSinceReport[board] = 0f;
            thighAngleRad[board] = 0f;
            lastTopTime[board] = 0.0;
            lastTopResetTime[board] = double.NegativeInfinity;
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
