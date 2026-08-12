using System;
using UnityEngine;

/// <summary>
/// VR peak-event controller with:
/// 1. avatar-relative peak anchoring,
/// 2. world-space stance locking,
/// 3. missing-event safe landing,
/// 4. short smooth anchor correction to avoid visible snapping,
/// 5. opposite-leg support enforcement so both feet are not airborne together.
/// </summary>
[DefaultExecutionOrder(-50)]
public sealed class PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth
    : MonoBehaviour
{
    public event Action<FootLandingEvent> FootLanded;
    [Header("Input")]
    [SerializeField] private LegPeakEventReceiver receiver;

    [Header("Foot Targets")]
    [SerializeField] private Transform leftFootTarget;
    [SerializeField] private Transform rightFootTarget;

    [Header("Spatial Anchors")]
    [Tooltip("Assign Avatar_LegIK_Test.")]
    [SerializeField] private Transform peakAnchorTransform;

    [Tooltip("Assign CenterEyeAnchor.")]
    [SerializeField] private Transform directionTransform;

    [Header("HMD Displacement")]
    [Min(0f)]
    [SerializeField] private float displacementScale = 1f;

    [Tooltip("Horizontal HMD movement below this value is treated as stationary.")]
    [Min(0.001f)]
    [SerializeField] private float minimumReferenceDisplacement = 0.03f;

    [Tooltip("Limits abnormal tracking or recentering jumps.")]
    [Min(0.01f)]
    [SerializeField] private float maximumPredictedDistance = 0.30f;

    [Header("Step Length Smoothing")]
    [Tooltip("Higher values preserve more of the previous filtered step length. Eyal suggested starting with 0.7.")]
    [Range(0f, 0.99f)]
    [SerializeField] private float stepLengthSmoothingAlpha = 0.70f;

    [Tooltip("Initialise the first valid filtered distance directly from the first valid raw estimate.")]
    [SerializeField] private bool initialiseFilterFromFirstEstimate = true;

    [Header("Timing")]
    [Min(0.2f)]
    [SerializeField] private float defaultSameLegInterval = 1.0f;

    [Min(0.2f)]
    [SerializeField] private float minimumSameLegInterval = 0.70f;

    [Min(0.2f)]
    [SerializeField] private float maximumSameLegInterval = 1.30f;

    [Header("Foot Path")]
    [Min(0.001f)]
    [SerializeField] private float peakHeight = 0.08f;

    [Range(0.05f, 0.49f)]
    [SerializeField] private float groundStartPhase = 0.25f;

    [Range(0.51f, 0.95f)]
    [SerializeField] private float groundEndPhase = 0.75f;

    [SerializeField]
    private AnimationCurve trajectorySmoothingCurve =
        AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

    [Header("Smooth Anchor Correction")]
    [Tooltip("Time used to blend from the old target position to the new avatar-relative peak.")]
    [Min(0f)]
    [SerializeField] private float anchorCorrectionDuration = 0.10f;

    [Tooltip("Corrections below this distance are applied directly.")]
    [Min(0f)]
    [SerializeField] private float directCorrectionThreshold = 0.005f;

    [SerializeField]
    private AnimationCurve anchorCorrectionCurve =
        AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

    [Header("Prediction-to-Prediction Blend")]
    [Tooltip("Time used to blend the previously predicted trajectory into the new trajectory after a peak event.")]
    [Min(0f)]
    [SerializeField] private float predictionBlendDuration = 0.05f;

    [SerializeField]
    private AnimationCurve predictionBlendCurve =
        AnimationCurve.EaseInOut(0f, 0f, 1f, 1f);

    [Header("Missing Event Recovery")]
    [Min(0f)]
    [SerializeField] private float eventTimeoutGrace = 0.20f;

    [Min(0.05f)]
    [SerializeField] private float fallbackLandingDuration = 0.20f;

    [SerializeField] private bool logFallbackLanding = true;

    [Header("Opposite Leg Support")]
    [Tooltip("When one leg receives a peak event, force the opposite foot onto the ground.")]
    [SerializeField] private bool enforceOppositeLegSupport = true;

    [Tooltip("Write a Console message whenever the opposite leg is forced to support.")]
    [SerializeField] private bool logOppositeSupport = true;

    [Header("Landing Event Detection")]
    [Tooltip("Write one Console message when a predicted foot landing is detected.")]
    [SerializeField] private bool logLandingEvents = true;

    [Header("Calibration")]
    [SerializeField] private bool keepInitialFootOffsets = true;

    [SerializeField] private Vector3 leftLocalHorizontalOffset;
    [SerializeField] private Vector3 rightLocalHorizontalOffset;

    [Header("Diagnostics")]
    [SerializeField] private bool logPredictions = true;

    [SerializeField] private float leftEstimatedInterval;
    [SerializeField] private float rightEstimatedInterval;
    [SerializeField] private float leftRawEstimatedDistance;
    [SerializeField] private float rightRawEstimatedDistance;
    [SerializeField] private float leftEstimatedDistance;
    [SerializeField] private float rightEstimatedDistance;

    [SerializeField] private float leftPhase;
    [SerializeField] private float rightPhase;

    [SerializeField] private float leftAnchorCorrection;
    [SerializeField] private float rightAnchorCorrection;

    [SerializeField] private bool leftAnchorBlendActive;
    [SerializeField] private bool rightAnchorBlendActive;

    [SerializeField] private bool leftPredictionBlendActive;
    [SerializeField] private bool rightPredictionBlendActive;
    [SerializeField] private float leftPredictionBlendProgress;
    [SerializeField] private float rightPredictionBlendProgress;

    [SerializeField] private bool leftFallbackLandingActive;
    [SerializeField] private bool rightFallbackLandingActive;

    [SerializeField] private int leftFallbackLandingCount;
    [SerializeField] private int rightFallbackLandingCount;

    [SerializeField] private int leftForcedSupportCount;
    [SerializeField] private int rightForcedSupportCount;

    [SerializeField] private int leftPredictedLandingCount;
    [SerializeField] private int rightPredictedLandingCount;

    [SerializeField] private double lastLeftLandingTime;
    [SerializeField] private double lastRightLandingTime;

    [SerializeField] private Vector3 lastLeftLandingPosition;
    [SerializeField] private Vector3 lastRightLandingPosition;

    private readonly LegState leftState = new LegState();
    private readonly LegState rightState = new LegState();

    private float leftGroundY;
    private float rightGroundY;
    private bool calibrated;

    private GaitPhase currentGaitPhase = GaitPhase.StartupIdle;
    private double gaitPhaseStartTime;
    private PeakLegSide? expectedPeakLeg;
    private PeakLegSide? lastAcceptedPeakLeg;
    private double lastAcceptedPeakTime;
    private bool hasAnyAcceptedPeak;
    private bool hasCrossLegInterval;
    private float lastCrossLegInterval;
    private float estimatedGlobalCycleDuration;
    private int gaitTransitionCount;
    private int sameLegRepeatEventCount;
    private PeakLegSide? lastAlternatingPeakLeg;
    private double lastAlternatingPeakTime;
    private float currentQuarterDuration;
    private float globalPhaseProgress;
    private PeakLegSide globalMovingLeg;
    private PeakLegSide globalSupportLeg;
    private Vector3 globalMotionStartPosition;
    private Vector3 globalMotionEndPosition;
    private Vector3 globalSupportPosition;
    private Vector3 globalPredictedOppositePeakPosition;
    private int earlyExpectedEventCount;
    private int ignoredUnexpectedEventCount;
    private bool pendingExpectedPeakReceived;
    private LegPeakEventReceiver.AppliedPeakEvent pendingExpectedPeakEvent;
    private float pendingEarlyAmount;
    private int consumedPendingEventCount;
    private int earlyRejectedEventCount;
    private int lastAcceptedPeakSequence;
    private bool hasConsumedPendingSequence;
    private PeakLegSide lastConsumedPendingLeg;
    private int lastConsumedPendingSequence;
    private bool isConsumingPendingEvent;
    private double pendingConsumptionMotionStartTime;

    // Read-only runtime diagnostics for UI. These properties do not modify
    // controller state and avoid reflection or per-frame object searches.
    public float LeftEstimatedInterval => leftEstimatedInterval;
    public float RightEstimatedInterval => rightEstimatedInterval;
    public float LeftRawEstimatedDistance => leftRawEstimatedDistance;
    public float RightRawEstimatedDistance => rightRawEstimatedDistance;
    public float LeftEstimatedDistance => leftEstimatedDistance;
    public float RightEstimatedDistance => rightEstimatedDistance;
    public bool LeftPredictionBlendActive => leftPredictionBlendActive;
    public bool RightPredictionBlendActive => rightPredictionBlendActive;
    public float LeftPredictionBlendProgress => leftPredictionBlendProgress;
    public float RightPredictionBlendProgress => rightPredictionBlendProgress;
    public string CurrentGlobalGaitPhase => currentGaitPhase.ToString();
    public double GaitPhaseStartTime => gaitPhaseStartTime;
    public PeakLegSide? ExpectedPeakLeg => expectedPeakLeg;
    public bool HasAnyAcceptedPeak => hasAnyAcceptedPeak;
    public PeakLegSide? LastAcceptedPeakLeg => lastAcceptedPeakLeg;
    public double LastAcceptedPeakTime => lastAcceptedPeakTime;
    public bool HasCrossLegInterval => hasCrossLegInterval;
    public float LastCrossLegInterval => lastCrossLegInterval;
    public float EstimatedGlobalCycleDuration => estimatedGlobalCycleDuration;
    public int GaitTransitionCount => gaitTransitionCount;
    public int SameLegRepeatEventCount => sameLegRepeatEventCount;
    public float CurrentQuarterDuration => currentQuarterDuration;
    public float GlobalPhaseProgress => globalPhaseProgress;
    public PeakLegSide GlobalMovingLeg => globalMovingLeg;
    public bool PeakHoldActive =>
        currentGaitPhase == GaitPhase.RightPeakHold ||
        currentGaitPhase == GaitPhase.LeftPeakHold;
    public int EarlyExpectedEventCount => earlyExpectedEventCount;
    public int IgnoredUnexpectedEventCount => ignoredUnexpectedEventCount;
    public bool PendingExpectedEventActive => pendingExpectedPeakReceived;
    public PeakLegSide? PendingExpectedPeakLeg =>
        pendingExpectedPeakReceived
            ? pendingExpectedPeakEvent.Leg
            : (PeakLegSide?)null;
    public float PendingEarlyAmount => pendingEarlyAmount;
    public int ConsumedPendingEventCount => consumedPendingEventCount;
    public int EarlyRejectedEventCount => earlyRejectedEventCount;


    /// <summary>
    /// Returns the currently predicted Current Peak, Landing and Next Peak
    /// positions for one leg. This is a read-only diagnostics interface.
    /// </summary>
    public bool TryGetTrajectoryPoints(
        PeakLegSide leg,
        out Vector3 currentPeak,
        out Vector3 landing,
        out Vector3 nextPeak)
    {
        LegState state =
            leg == PeakLegSide.Left
                ? leftState
                : rightState;

        currentPeak = state.CurrentPeakPosition;
        landing = state.LandingPosition;
        nextPeak = state.NextPeakPosition;

        return state.IsActive || state.FallbackLandingActive;
    }

    //初始化阶段

    // ===== Unity 生命周期：初始化 Ground Y 与事件订阅 =====
    // Awake 必须在任何步态计算前保存左右 Foot Target 的初始世界高度；
    // 后续 Landing、Fallback 和支撑脚落地都把它们当作固定地面基准。
    private void Awake()//记录左右脚的初始世界 Y
    {
        if (receiver == null)
        {
            receiver = FindFirstObjectByType<LegPeakEventReceiver>();
        }

        if (leftFootTarget != null)
        {
            leftGroundY = leftFootTarget.position.y;
        }

        if (rightFootTarget != null)
        {
            rightGroundY = rightFootTarget.position.y;
        }
    }

    private void Start()
    {
        CalibrateFootOffsets();
        //它记录两只 Foot Target 相对 Avatar Root 的水平偏移。
        //这使 Startup Idle 和 Post - cycle Idle 能做到：Avatar 在水平面移动时，站立的双脚仍保持在 Avatar 下方。
    }

    private void OnEnable()
    {
        if (receiver != null)
        {
            receiver.PeakApplied += HandlePeakApplied;
        }
    }

    // ===== 每帧总调度入口 =====
    // 全局 T/4 阶段优先；否则更新单腿兼容/Recovery/Fallback 路径，
    // 最后在完全静止时让双脚继续跟随 Avatar 的水平位置。
    private void Update()
    {
        double now = Time.realtimeSinceStartupAsDouble;

        if (IsGlobalMotionPhase())
        {
            UpdateGlobalGaitMotion(now);
            return;
        }

        UpdateLeg(
            PeakLegSide.Left,
            leftState,
            leftFootTarget,
            leftGroundY,
            now,
            out leftPhase);

        UpdateLeg(
            PeakLegSide.Right,
            rightState,
            rightFootTarget,
            rightGroundY,
            now,
            out rightPhase);

        if (ShouldPostTrajectoryAvatarFollow())
        {
            UpdateAvatarRelativeStandingFoot(
                PeakLegSide.Left,
                leftFootTarget,
                leftGroundY);

            UpdateAvatarRelativeStandingFoot(
                PeakLegSide.Right,
                rightFootTarget,
                rightGroundY);
        }

        UpdateDiagnosticGaitPhase(now);
    }

    [ContextMenu("Calibrate Foot Offsets")]
    // ===== Foot Target 水平锚点校准 =====
    // 只保存左右脚相对 Avatar Root 的 X/Z 偏移，Y 始终由 Ground Y 管理。
    public void CalibrateFootOffsets()
    {
        if (peakAnchorTransform == null)
        {
            Debug.LogWarning(
                "[PeakTrajectorySmooth] Peak Anchor Transform is not assigned.",
                this);

            calibrated = false;
            return;
        }

        if (leftFootTarget != null)
        {
            leftLocalHorizontalOffset =
                peakAnchorTransform.InverseTransformPoint(
                    leftFootTarget.position);

            leftLocalHorizontalOffset.y = 0f;
        }

        if (rightFootTarget != null)
        {
            rightLocalHorizontalOffset =
                peakAnchorTransform.InverseTransformPoint(
                    rightFootTarget.position);

            rightLocalHorizontalOffset.y = 0f;
        }

        calibrated = true;
    }

  
    // ===== Peak 事件核心入口 =====
    // 一个已统一的 Left/Right Peak 到达后，本方法依次完成：
    // 事件合法性判断 -> 周期/步长估计 -> 三个轨迹点生成 ->
    // 首次 Peak 对齐或后续 Prediction Blend -> 启动全局 T/4 Lowering。
    private void HandlePeakApplied(
        LegPeakEventReceiver.AppliedPeakEvent peakEvent)
    {
        LegState state = peakEvent.Leg == PeakLegSide.Left
            ? leftState
            : rightState;

        Transform target = peakEvent.Leg == PeakLegSide.Left
            ? leftFootTarget
            : rightFootTarget;

        float groundY = peakEvent.Leg == PeakLegSide.Left
            ? leftGroundY
            : rightGroundY;
        //找到对应腿


        if (target == null)
        {
            Debug.LogWarning(
                $"[PeakTrajectorySmooth] No target assigned for {peakEvent.Leg}.",
                this);

            return;
        }

        if (!calibrated)
        {
            CalibrateFootOffsets();
        }

        GaitPhase phaseBeforeEvent = currentGaitPhase;

        if (phaseBeforeEvent == GaitPhase.StartupIdle)
        {
            ClearPendingExpectedPeak(
                "new first event",
                true);
        }

        PeakEventDisposition disposition =
            ClassifyGlobalPeakEvent(peakEvent);//检查事件在当前阶段是否合法

        if (disposition == PeakEventDisposition.CacheExpectedEarly)
        {
            CacheExpectedEarlyPeak(peakEvent);
            return;
        }

        if (disposition == PeakEventDisposition.RejectOrIgnore)
        {
            return;
        }

        bool isFirstPeakForThisLeg = //判断是否为该腿第一次 Peak
            !state.HasPreviousEvent &&
            !isConsumingPendingEvent;

        double motionEventTime = isConsumingPendingEvent
            ? pendingConsumptionMotionStartTime
            : peakEvent.UnityTimestamp;

        if (enforceOppositeLegSupport)
        {
            EnforceOppositeLegSupport(
                peakEvent.Leg,
                peakEvent.UnityTimestamp);
        }
        //计算同腿周期，第一次没有历史数据，使用：
        float interval = defaultSameLegInterval;

        if (state.HasPreviousEvent)
        {
            interval = Mathf.Clamp(
                (float)(peakEvent.UnityTimestamp -
                        state.PreviousUnityEventTime),
                minimumSameLegInterval,
                maximumSameLegInterval);//限制
            //interval =当前同腿Peak时间-上一次同腿Peak时间
        }

        Vector3 horizontalDelta = Vector3.zero;

        if (state.HasPreviousReference)
        {
            horizontalDelta =
                peakEvent.ReferencePosition -
                state.PreviousReferencePosition;

            horizontalDelta.y = 0f;
            //计算 HMD 位移，并删除 Y，只用 HMD 水平位移，不把低头、蹲下等垂直运动算成步长。
        }

        Vector3 direction =
            GetHorizontalDirection(horizontalDelta);

        float rawPredictedDistance =
            CalculatePredictedDistance(horizontalDelta);

        float predictedDistance =
            ApplyStepLengthSmoothing(
                state,
                rawPredictedDistance,
                state.HasPreviousReference);
   

        Vector3 oldTargetPosition = target.position;

        PredictionSnapshot previousPrediction = 
            isFirstPeakForThisLeg
                ? null
                : CapturePredictionSnapshot(
                    state,
                    oldTargetPosition,
                    motionEventTime);

        Vector3 currentPeak =
            BuildAnchoredPeakPosition(
                peakEvent.Leg,
                oldTargetPosition,
                groundY);

        float correctionMagnitude =
            Vector3.Distance(oldTargetPosition, currentPeak);

        Vector3 landing =
            currentPeak +
            direction * (predictedDistance * 0.5f);

        landing.y = groundY;

        Vector3 nextPeak =
            currentPeak +
            direction * predictedDistance;

        nextPeak.y = groundY + peakHeight;

        state.FallbackLandingActive = false;
        SetFallbackActive(peakEvent.Leg, false);

        state.IsActive = true;
        state.CycleStartUnityTime = motionEventTime;
        state.CycleDuration = interval;
        state.CurrentPeakPosition = currentPeak;
        state.LandingPosition = landing;
        state.NextPeakPosition = nextPeak;
        state.LandingEventFired = false;
  
        if (isFirstPeakForThisLeg)
        {
            state.PreviousPrediction = null;
            state.PredictionBlendActive = false;
            SetPredictionBlendActive(
                peakEvent.Leg,
                false,
                1f);

            state.AnchorCorrectionActive = false;
            SetAnchorBlendActive(
                peakEvent.Leg,
                false);
        }
        else
        {
            state.PreviousPrediction = previousPrediction;
            state.PredictionBlendStartTime =
                motionEventTime;

            state.PredictionBlendActive =
                predictionBlendDuration > 0f &&
                previousPrediction.IsValid;

            SetPredictionBlendActive(
                peakEvent.Leg,
                state.PredictionBlendActive,
                0f);



            // The prediction-to-prediction blend replaces the old static
            // position-offset correction whenever an old prediction is available.
            state.AnchorCorrectionOffset =
                oldTargetPosition - currentPeak;

            state.AnchorCorrectionStartTime =
                motionEventTime;

            state.AnchorCorrectionActive =
                !state.PredictionBlendActive &&
                anchorCorrectionDuration > 0f &&
                correctionMagnitude > directCorrectionThreshold;

            SetAnchorBlendActive(
                peakEvent.Leg,
                state.AnchorCorrectionActive);
        }

        state.PreviousUnityEventTime =
            peakEvent.UnityTimestamp;

        state.HasPreviousEvent = true;

        state.PreviousReferencePosition =
            peakEvent.ReferencePosition;

        state.HasPreviousReference = true;

          target.position = isFirstPeakForThisLeg
            ? currentPeak
            : oldTargetPosition;

        if (peakEvent.Leg == PeakLegSide.Left)
        {
            leftEstimatedInterval = interval;
            leftRawEstimatedDistance = rawPredictedDistance;
            leftEstimatedDistance = predictedDistance;
            leftAnchorCorrection = correctionMagnitude;
        }
        else
        {
            rightEstimatedInterval = interval;
            rightRawEstimatedDistance = rawPredictedDistance;
            rightEstimatedDistance = predictedDistance;
            rightAnchorCorrection = correctionMagnitude;
        }

        if (logPredictions)
        {
            Debug.Log(
                $"[PeakTrajectorySmooth] leg={peakEvent.Leg}, " +
                $"interval={interval:F3}s, " +
                $"rawDistance={rawPredictedDistance:F3}m, " +
                $"filteredDistance={predictedDistance:F3}m, " +
                $"alpha={stepLengthSmoothingAlpha:F2}, " +
                $"referenceDelta={horizontalDelta.magnitude:F3}m, " +
                $"anchorCorrection={correctionMagnitude:F3}m, " +
                $"anchorBlendDuration={anchorCorrectionDuration:F3}s, " +
                $"predictionBlendDuration={predictionBlendDuration:F3}s",
                this);
        }

        RecordGlobalPeakEvent(peakEvent);

        BeginGlobalLowering(
            peakEvent.Leg,
            peakEvent.UnityTimestamp,
            direction,
            predictedDistance,
            phaseBeforeEvent);
    }

    // ===== Peak 合法性与时序过滤 =====
    // 根据当前全局阶段把事件分为：立即接受、提前缓存、拒绝/忽略。
    // Wrong Event Filter 与 Early Pending 的入口都集中在这里。
    private PeakEventDisposition ClassifyGlobalPeakEvent(
        LegPeakEventReceiver.AppliedPeakEvent peakEvent)
    {
        PeakLegSide eventLeg = peakEvent.Leg;

        if (currentGaitPhase == GaitPhase.StartupIdle ||
            currentGaitPhase == GaitPhase.AvatarFollowIdle ||
            currentGaitPhase == GaitPhase.Recovery)
        {
            return PeakEventDisposition.AcceptNow;
        }

        if (currentGaitPhase == GaitPhase.RightPeakHold)
        {
            return ValidatePeakHoldEvent(
                eventLeg,
                PeakLegSide.Right);
        }

        if (currentGaitPhase == GaitPhase.LeftPeakHold)
        {
            return ValidatePeakHoldEvent(
                eventLeg,
                PeakLegSide.Left);
        }

        bool isRising =
            currentGaitPhase == GaitPhase.RightRising ||
            currentGaitPhase == GaitPhase.LeftRising;

        if (isRising &&
            expectedPeakLeg.HasValue &&
            expectedPeakLeg.Value == eventLeg)
        {
            return CanCacheExpectedEarlyPeak(peakEvent)
                ? PeakEventDisposition.CacheExpectedEarly
                : PeakEventDisposition.RejectOrIgnore;
        }

        if (currentGaitPhase == GaitPhase.LeftLowering ||
            currentGaitPhase == GaitPhase.RightRising ||
            currentGaitPhase == GaitPhase.RightLowering ||
            currentGaitPhase == GaitPhase.LeftRising)
        {
            ignoredUnexpectedEventCount++;

            if (logPredictions)
            {
                Debug.Log(
                    $"[GlobalGaitMotion] Unexpected event " +
                    $"ignored during {currentGaitPhase}, " +
                    $"event={eventLeg}.",
                    this);
            }

            return PeakEventDisposition.RejectOrIgnore;
        }

        return PeakEventDisposition.AcceptNow;
    }

    private PeakEventDisposition ValidatePeakHoldEvent(
        PeakLegSide receivedLeg,
        PeakLegSide requiredLeg)
    {
        if (receivedLeg == requiredLeg)
        {
            return PeakEventDisposition.AcceptNow;
        }

        ignoredUnexpectedEventCount++;
        sameLegRepeatEventCount++;

        if (logPredictions)
        {
            Debug.Log(
                $"[GlobalGaitMotion] Expected={requiredLeg}, " +
                $"received={receivedLeg}, event ignored during PeakHold.",
                this);
        }

        return PeakEventDisposition.RejectOrIgnore;
    }

    private bool CanCacheExpectedEarlyPeak(
        LegPeakEventReceiver.AppliedPeakEvent peakEvent)
    {
        bool hasValidTime =
            peakEvent.UnityTimestamp > 0.0 &&
            !double.IsNaN(peakEvent.UnityTimestamp) &&
            !double.IsInfinity(peakEvent.UnityTimestamp);

        bool duplicatesLastAccepted =
            hasAnyAcceptedPeak &&
            lastAcceptedPeakLeg.HasValue &&
            lastAcceptedPeakLeg.Value == peakEvent.Leg &&
            lastAcceptedPeakSequence == peakEvent.Sequence;

        bool duplicatesConsumedPending =
            hasConsumedPendingSequence &&
            lastConsumedPendingLeg == peakEvent.Leg &&
            lastConsumedPendingSequence == peakEvent.Sequence;

        if (pendingExpectedPeakReceived ||
            !hasValidTime ||
            duplicatesLastAccepted ||
            duplicatesConsumedPending)
        {
            earlyRejectedEventCount++;

            if (logPredictions)
            {
                Debug.Log(
                    $"[GlobalGaitMotion] Early expected event rejected: " +
                    $"pending-or-duplicate, event={peakEvent.Leg}, " +
                    $"sequence={peakEvent.Sequence}.",
                    this);
            }

            return false;
        }

        double predictedArrivalTime =
            gaitPhaseStartTime + currentQuarterDuration;

        double earlyAmount =
            predictedArrivalTime - peakEvent.UnityTimestamp;

        if (earlyAmount > currentQuarterDuration + 0.001)
        {
            earlyRejectedEventCount++;

            if (logPredictions)
            {
                Debug.Log(
                    "[GlobalGaitMotion] Early expected event " +
                    "rejected: too early.",
                    this);
            }

            return false;
        }

        return true;
    }

    private void CacheExpectedEarlyPeak(
        LegPeakEventReceiver.AppliedPeakEvent peakEvent)
    {
        double predictedArrivalTime =
            gaitPhaseStartTime + currentQuarterDuration;

        pendingExpectedPeakReceived = true;
        pendingExpectedPeakEvent = peakEvent;
        pendingEarlyAmount = Mathf.Max(
            0f,
            (float)(predictedArrivalTime -
                    peakEvent.UnityTimestamp));
        earlyExpectedEventCount++;

        if (logPredictions)
        {
            Debug.Log(
                $"[GlobalGaitMotion] Early expected " +
                $"{peakEvent.Leg} cached during " +
                $"{currentGaitPhase}.",
                this);
        }
    }

    // ===== 全局周期估计 =====
    // 记录交替腿 Peak 的时间关系，更新 cross-leg interval、
    // global cycle duration 与 T/4 所需的 quarter duration。
    private void RecordGlobalPeakEvent(
        LegPeakEventReceiver.AppliedPeakEvent peakEvent)
    {
        PeakLegSide eventLeg = peakEvent.Leg;
        double eventTime = peakEvent.UnityTimestamp;

        lastAcceptedPeakLeg = eventLeg;
        lastAcceptedPeakTime = eventTime;
        lastAcceptedPeakSequence = peakEvent.Sequence;

        if (!hasAnyAcceptedPeak)
        {
            hasAnyAcceptedPeak = true;
            lastAlternatingPeakLeg = eventLeg;
            lastAlternatingPeakTime = eventTime;

            RecordConfirmedConceptualPhase(
                eventLeg,
                eventTime,
                "first event");

            return;
        }

        if (currentGaitPhase == GaitPhase.Recovery ||
            currentGaitPhase == GaitPhase.AvatarFollowIdle)
        {
            lastAlternatingPeakLeg = eventLeg;
            lastAlternatingPeakTime = eventTime;

            RecordConfirmedConceptualPhase(
                eventLeg,
                eventTime,
                "event recovery");

            return;
        }

        if (lastAlternatingPeakLeg.HasValue &&
            lastAlternatingPeakLeg.Value == eventLeg)
        {
            sameLegRepeatEventCount++;

            if (logPredictions)
            {
                string expected = expectedPeakLeg.HasValue
                    ? expectedPeakLeg.Value.ToString()
                    : "None";

                Debug.Log(
                    $"[GlobalGait] Expected={expected}, " +
                    $"received={eventLeg}, same-leg repeat recorded.",
                    this);
            }

            return;
        }

        if (lastAlternatingPeakLeg.HasValue)
        {
            double interval =
                eventTime - lastAlternatingPeakTime;

            if (interval > 0.0)
            {
                lastCrossLegInterval = (float)interval;
                hasCrossLegInterval = true;
                estimatedGlobalCycleDuration =
                    Mathf.Clamp(
                        lastCrossLegInterval * 2f,
                        minimumSameLegInterval,
                        maximumSameLegInterval);
            }
        }

        lastAlternatingPeakLeg = eventLeg;
        lastAlternatingPeakTime = eventTime;

        RecordConfirmedConceptualPhase(
            eventLeg,
            eventTime,
            "cross-leg event");
    }

    private void RecordConfirmedConceptualPhase(
        PeakLegSide eventLeg,
        double eventTime,
        string reason)
    {
        GaitPhase nextPhase =
            eventLeg == PeakLegSide.Left
                ? GaitPhase.LeftLowering
                : GaitPhase.RightLowering;

        PeakLegSide nextExpectedLeg =
            eventLeg == PeakLegSide.Left
                ? PeakLegSide.Right
                : PeakLegSide.Left;

        SetDiagnosticGaitPhase(
            nextPhase,
            nextExpectedLeg,
            eventTime,
            $"event={eventLeg}, {reason}");
    }

    private void UpdateDiagnosticGaitPhase(double now)
    {
        if (!hasAnyAcceptedPeak)
        {
            SetDiagnosticGaitPhase(
                GaitPhase.StartupIdle,
                null,
                now,
                "no accepted peak");

            return;
        }

        if (leftState.FallbackLandingActive ||
            rightState.FallbackLandingActive)
        {
            SetDiagnosticGaitPhase(
                GaitPhase.Recovery,
                null,
                now,
                "fallback active");

            return;
        }

        if (AreBothLegsQuiescent())
        {
            SetDiagnosticGaitPhase(
                GaitPhase.AvatarFollowIdle,
                null,
                now,
                "both legs quiescent");
        }
    }

    private void SetDiagnosticGaitPhase(
        GaitPhase nextPhase,
        PeakLegSide? nextExpectedLeg,
        double transitionTime,
        string reason)
    {
        if (pendingExpectedPeakReceived &&
            (!nextExpectedLeg.HasValue ||
             pendingExpectedPeakEvent.Leg != nextExpectedLeg.Value))
        {
            ClearPendingExpectedPeak(
                nextPhase.ToString(),
                true);
        }

        if (currentGaitPhase == nextPhase &&
            expectedPeakLeg == nextExpectedLeg)
        {
            return;
        }

        GaitPhase previousPhase = currentGaitPhase;
        currentGaitPhase = nextPhase;
        expectedPeakLeg = nextExpectedLeg;
        gaitPhaseStartTime = transitionTime;
        gaitTransitionCount++;

        if (logPredictions)
        {
            Debug.Log(
                $"[GlobalGait] {previousPhase} -> {nextPhase}, " +
                $"expected={FormatExpectedPeakLeg(nextExpectedLeg)}, " +
                reason,
                this);
        }
    }

    private static string FormatExpectedPeakLeg(
        PeakLegSide? leg)
    {
        return leg.HasValue
            ? leg.Value.ToString()
            : "None";
    }

    private bool IsGlobalMotionPhase()
    {
        return currentGaitPhase == GaitPhase.LeftLowering ||
               currentGaitPhase == GaitPhase.RightRising ||
               currentGaitPhase == GaitPhase.RightPeakHold ||
               currentGaitPhase == GaitPhase.RightLowering ||
               currentGaitPhase == GaitPhase.LeftRising ||
               currentGaitPhase == GaitPhase.LeftPeakHold;
    }

    // ===== T/4 阶段 1：当前 Peak 腿下降 =====
    // 从 CurrentPeakPosition 移动到 LandingPosition；另一条腿保持支撑。
    private void BeginGlobalLowering(
        PeakLegSide confirmedLeg,
        double eventTime,
        Vector3 direction,
        float predictedDistance,
        GaitPhase phaseBeforeEvent)
    {
        LegState movingState = GetLegState(confirmedLeg);
        Transform movingTarget = GetFootTarget(confirmedLeg);
        PeakLegSide oppositeLeg = GetOppositeLeg(confirmedLeg);
        Transform supportTarget = GetFootTarget(oppositeLeg);

        if (movingTarget == null || supportTarget == null)
        {
            BeginGlobalRecovery(eventTime, "missing foot target");
            return;
        }

        currentQuarterDuration =
            ResolveQuarterDuration(movingState);

        globalMovingLeg = confirmedLeg;
        globalSupportLeg = oppositeLeg;
        globalMotionStartPosition =
            movingState.CurrentPeakPosition;
        globalMotionEndPosition =
            movingState.LandingPosition;

        globalSupportPosition = supportTarget.position;
        globalSupportPosition.y = GetGroundY(oppositeLeg);
        supportTarget.position = globalSupportPosition;

        globalPredictedOppositePeakPosition =
            BuildOppositePredictedPeak(
                oppositeLeg,
                direction,
                predictedDistance);

        GaitPhase loweringPhase =
            confirmedLeg == PeakLegSide.Left
                ? GaitPhase.LeftLowering
                : GaitPhase.RightLowering;

        PeakLegSide nextExpectedLeg = oppositeLeg;

        double motionStartTime = isConsumingPendingEvent
            ? pendingConsumptionMotionStartTime
            : eventTime;

        gaitPhaseStartTime = motionStartTime;
        globalPhaseProgress = 0f;

        SetDiagnosticGaitPhase(
            loweringPhase,
            nextExpectedLeg,
            motionStartTime,
            $"confirmed {confirmedLeg}");

        if (logPredictions)
        {
            Debug.Log(
                $"[GlobalGaitMotion] {phaseBeforeEvent} -> " +
                $"{loweringPhase}, confirmed {confirmedLeg}, " +
                $"Q={currentQuarterDuration:F3}s.",
                this);
        }
    }

    private float ResolveQuarterDuration(LegState eventState)
    {
        float cycleDuration =
            hasCrossLegInterval &&
            estimatedGlobalCycleDuration > 0f
                ? estimatedGlobalCycleDuration
                : eventState.CycleDuration > 0f
                    ? eventState.CycleDuration
                    : defaultSameLegInterval;

        cycleDuration = Mathf.Clamp(
            cycleDuration,
            minimumSameLegInterval,
            maximumSameLegInterval);

        return Mathf.Max(0.001f, cycleDuration * 0.25f);
    }

    private Vector3 BuildOppositePredictedPeak(
        PeakLegSide oppositeLeg,
        Vector3 direction,
        float predictedDistance)
    {
        Transform oppositeTarget = GetFootTarget(oppositeLeg);
        Vector3 fallbackPosition = oppositeTarget != null
            ? oppositeTarget.position
            : Vector3.zero;

        Vector3 predictedPeak =
            BuildAnchoredPeakPosition(
                oppositeLeg,
                fallbackPosition,
                GetGroundY(oppositeLeg));

        predictedPeak +=
            direction * (predictedDistance * 0.5f);

        predictedPeak.y =
            GetGroundY(oppositeLeg) + peakHeight;

        return predictedPeak;
    }

    // ===== 全局 T/4 状态机总调度 =====
    // 按当前 GaitPhase 分派 Lowering、Rising、Peak Hold 或 Recovery。
    // 这里决定本帧哪只 Foot Target 被写入以及何时切换阶段。
    private void UpdateGlobalGaitMotion(double now)
    {
        switch (currentGaitPhase)
        {
            case GaitPhase.LeftLowering:
                UpdateGlobalLowering(PeakLegSide.Left, now);
                break;

            case GaitPhase.RightLowering:
                UpdateGlobalLowering(PeakLegSide.Right, now);
                break;

            case GaitPhase.RightRising:
                UpdateGlobalRising(PeakLegSide.Right, now);
                break;

            case GaitPhase.LeftRising:
                UpdateGlobalRising(PeakLegSide.Left, now);
                break;

            case GaitPhase.RightPeakHold:
                UpdateGlobalPeakHold(PeakLegSide.Right, now);
                break;

            case GaitPhase.LeftPeakHold:
                UpdateGlobalPeakHold(PeakLegSide.Left, now);
                break;
        }
    }

    private void UpdateGlobalLowering(
        PeakLegSide loweringLeg,
        double now)
    {
        LegState state = GetLegState(loweringLeg);
        Transform target = GetFootTarget(loweringLeg);
        Transform supportTarget = GetFootTarget(globalSupportLeg);

        if (target == null || supportTarget == null)
        {
            BeginGlobalRecovery(now, "missing foot target");
            return;
        }

        globalPhaseProgress = CalculateGlobalPhaseProgress(now);

        Vector3 nominalPosition =
            Vector3.Lerp(
                globalMotionStartPosition,
                globalMotionEndPosition,
                EvaluateTrajectoryCurve(globalPhaseProgress));

        Vector3 correctedPosition =
            ApplyAnchorCorrection(
                loweringLeg,
                state,
                nominalPosition,
                now);

        target.position =
            ApplyPredictionBlend(
                loweringLeg,
                state,
                correctedPosition,
                now);

        supportTarget.position = globalSupportPosition;

        if (globalPhaseProgress < 1f)
        {
            return;
        }

        target.position = globalMotionEndPosition;
        CompleteGlobalCorrection(loweringLeg, state);

        if (!state.LandingEventFired)
        {
            state.LandingEventFired = true;

            RaiseLandingEvent(
                loweringLeg,
                globalMotionEndPosition,
                now,
                LandingReason.Predicted);
        }

        BeginGlobalRising(
            GetOppositeLeg(loweringLeg),
            loweringLeg,
            now);
    }

    // ===== T/4 阶段 2：对侧腿上升 =====
    // 支撑腿保持世界锁定；moving leg 从地面移向预测的 Opposite Peak。
    private void BeginGlobalRising(
        PeakLegSide risingLeg,
        PeakLegSide supportLeg,
        double now)
    {
        Transform risingTarget = GetFootTarget(risingLeg);

        if (risingTarget == null)
        {
            BeginGlobalRecovery(now, "missing rising target");
            return;
        }

        globalMovingLeg = risingLeg;
        globalSupportLeg = supportLeg;
        globalSupportPosition =
            GetFootTarget(supportLeg).position;
        globalSupportPosition.y = GetGroundY(supportLeg);
        globalMotionStartPosition = risingTarget.position;
        globalMotionStartPosition.y = GetGroundY(risingLeg);
        risingTarget.position = globalMotionStartPosition;
        globalMotionEndPosition =
            globalPredictedOppositePeakPosition;

        GaitPhase risingPhase =
            risingLeg == PeakLegSide.Right
                ? GaitPhase.RightRising
                : GaitPhase.LeftRising;

        GaitPhase previousPhase = currentGaitPhase;
        gaitPhaseStartTime = now;
        globalPhaseProgress = 0f;

        SetDiagnosticGaitPhase(
            risingPhase,
            risingLeg,
            now,
            $"{supportLeg} landing completed");

        LogGlobalMotionTransition(previousPhase, risingPhase);
    }

    private void UpdateGlobalRising(
        PeakLegSide risingLeg,
        double now)
    {
        Transform target = GetFootTarget(risingLeg);
        Transform supportTarget = GetFootTarget(globalSupportLeg);

        if (target == null || supportTarget == null)
        {
            BeginGlobalRecovery(now, "missing foot target");
            return;
        }

        globalPhaseProgress = CalculateGlobalPhaseProgress(now);

        target.position =
            Vector3.Lerp(
                globalMotionStartPosition,
                globalMotionEndPosition,
                EvaluateTrajectoryCurve(globalPhaseProgress));

        supportTarget.position = globalSupportPosition;

        if (globalPhaseProgress < 1f)
        {
            return;
        }

        target.position = globalMotionEndPosition;
        BeginGlobalPeakHold(risingLeg, now);
    }

    // ===== T/4 阶段 3：Peak Hold =====
    // 上升腿到达预测 Peak 后保持，等待对应真实 Peak。
    // 提前到达的合法事件可由 Pending 机制在这里消费。
    private void BeginGlobalPeakHold(
        PeakLegSide peakLeg,
        double now)
    {
        GaitPhase holdPhase =
            peakLeg == PeakLegSide.Right
                ? GaitPhase.RightPeakHold
                : GaitPhase.LeftPeakHold;

        GaitPhase previousPhase = currentGaitPhase;
        gaitPhaseStartTime = now;
        globalPhaseProgress = 1f;

        SetDiagnosticGaitPhase(
            holdPhase,
            peakLeg,
            now,
            $"{peakLeg} reached predicted peak");

        LogGlobalMotionTransition(previousPhase, holdPhase);

        if (!pendingExpectedPeakReceived)
        {
            return;
        }

        if (pendingExpectedPeakEvent.Leg != peakLeg)
        {
            ClearPendingExpectedPeak(
                "expected leg changed",
                true);
            return;
        }

        ConsumePendingExpectedPeak(now);
    }

    // ===== Early Pending 消费 =====
    // 将 Rising 阶段提前收到并缓存的正确腿事件，在 Peak Hold 开始后正式应用。
    private void ConsumePendingExpectedPeak(double now)
    {
        LegPeakEventReceiver.AppliedPeakEvent pendingEvent =
            pendingExpectedPeakEvent;

        ClearPendingExpectedPeak(null, false);

        consumedPendingEventCount++;
        hasConsumedPendingSequence = true;
        lastConsumedPendingLeg = pendingEvent.Leg;
        lastConsumedPendingSequence = pendingEvent.Sequence;

        if (logPredictions)
        {
            Debug.Log(
                $"[GlobalGaitMotion] Pending {pendingEvent.Leg} " +
                $"consumed at {currentGaitPhase} -> " +
                $"{(pendingEvent.Leg == PeakLegSide.Right ? GaitPhase.RightLowering : GaitPhase.LeftLowering)}.",
                this);
        }

        isConsumingPendingEvent = true;
        pendingConsumptionMotionStartTime = now;

        try
        {
            HandlePeakApplied(pendingEvent);
        }
        finally
        {
            isConsumingPendingEvent = false;
        }
    }

    private void ClearPendingExpectedPeak(
        string reason,
        bool writeLog)
    {
        if (!pendingExpectedPeakReceived)
        {
            return;
        }

        pendingExpectedPeakReceived = false;
        pendingExpectedPeakEvent = default;
        pendingEarlyAmount = 0f;

        if (writeLog && logPredictions)
        {
            Debug.Log(
                $"[GlobalGaitMotion] Pending event cleared on {reason}.",
                this);
        }
    }

    private void UpdateGlobalPeakHold(
        PeakLegSide peakLeg,
        double now)
    {
        Transform peakTarget = GetFootTarget(peakLeg);
        Transform supportTarget = GetFootTarget(globalSupportLeg);

        if (peakTarget == null || supportTarget == null)
        {
            BeginGlobalRecovery(now, "missing foot target");
            return;
        }

        globalPhaseProgress = 1f;
        peakTarget.position = globalMotionEndPosition;
        supportTarget.position = globalSupportPosition;

        if (now - gaitPhaseStartTime < eventTimeoutGrace)
        {
            return;
        }

        if (logPredictions)
        {
            Debug.Log(
                "[GlobalGaitMotion] PeakHold timeout -> Recovery.",
                this);
        }

        BeginGlobalRecovery(now, "peak hold timeout");
    }

    private float CalculateGlobalPhaseProgress(double now)
    {
        return Mathf.Clamp01(
            (float)((now - gaitPhaseStartTime) /
                    Mathf.Max(0.001f, currentQuarterDuration)));
    }

    private void CompleteGlobalCorrection(
        PeakLegSide leg,
        LegState state)
    {
        state.AnchorCorrectionActive = false;
        SetAnchorBlendActive(leg, false);

        state.PredictionBlendActive = false;
        state.PreviousPrediction = null;
        SetPredictionBlendActive(leg, false, 1f);
    }

    // ===== 全局 Recovery =====
    // 当期望 Peak 超时或全局运动无法继续时，让活动脚安全回到各自 Ground Y，
    // 清理 Pending/Blend 状态，避免腿长期悬空。
    private void BeginGlobalRecovery(
        double now,
        string reason)
    {
        ClearPendingExpectedPeak("Recovery", true);

        SetDiagnosticGaitPhase(
            GaitPhase.Recovery,
            null,
            now,
            reason);

        globalPhaseProgress = 0f;

        PrepareLegForGlobalRecovery(
            PeakLegSide.Left,
            leftState,
            leftFootTarget,
            leftGroundY,
            now);

        PrepareLegForGlobalRecovery(
            PeakLegSide.Right,
            rightState,
            rightFootTarget,
            rightGroundY,
            now);
    }

    private void PrepareLegForGlobalRecovery(
        PeakLegSide leg,
        LegState state,
        Transform target,
        float groundY,
        double now)
    {
        state.IsActive = false;
        state.AnchorCorrectionActive = false;
        state.PredictionBlendActive = false;
        state.PreviousPrediction = null;

        SetAnchorBlendActive(leg, false);
        SetPredictionBlendActive(leg, false, 1f);

        if (target == null)
        {
            return;
        }

        bool isAboveGround =
            Mathf.Abs(target.position.y - groundY) >
            directCorrectionThreshold;

        if (isAboveGround)
        {
            StartFallbackLanding(
                leg,
                state,
                target,
                groundY,
                now);
        }
        else
        {
            state.FallbackLandingActive = false;
            SetFallbackActive(leg, false);
        }
    }

    private void LogGlobalMotionTransition(
        GaitPhase previousPhase,
        GaitPhase nextPhase)
    {
        if (!logPredictions)
        {
            return;
        }

        Debug.Log(
            $"[GlobalGaitMotion] {previousPhase} -> {nextPhase}.",
            this);
    }

    private LegState GetLegState(PeakLegSide leg)
    {
        return leg == PeakLegSide.Left
            ? leftState
            : rightState;
    }

    private Transform GetFootTarget(PeakLegSide leg)
    {
        return leg == PeakLegSide.Left
            ? leftFootTarget
            : rightFootTarget;
    }

    private float GetGroundY(PeakLegSide leg)
    {
        return leg == PeakLegSide.Left
            ? leftGroundY
            : rightGroundY;
    }

    private static PeakLegSide GetOppositeLeg(PeakLegSide leg)
    {
        return leg == PeakLegSide.Left
            ? PeakLegSide.Right
            : PeakLegSide.Left;
    }

    // ===== 单腿兼容、Fallback 与 Landing 更新路径 =====
    // 全局 T/4 未接管时，本方法仍负责旧轨迹兼容、Prediction Blend、
    // Anchor Correction、超时 Fallback Landing 以及 Landing 事件检测。
    private void UpdateLeg(
        PeakLegSide leg,
        LegState state,
        Transform target,
        float groundY,
        double now,
        out float phase)
    {
        phase = 0f;

        if (target == null)
        {
            return;
        }

        if (state.FallbackLandingActive)
        {
            UpdateFallbackLanding(
                leg,
                state,
                target,
                now);

            phase = 1f;
            return;
        }

        if (!state.IsActive)
        {
            if (!state.HasPreviousEvent &&
                !state.AnchorCorrectionActive &&
                !state.PredictionBlendActive)
            {
                UpdateAvatarRelativeStandingFoot(
                    leg,
                    target,
                    groundY);
            }

            return;
        }

        float duration =
            Mathf.Max(0.001f, state.CycleDuration);

        double elapsed =
            now - state.CycleStartUnityTime;

        phase = Mathf.Clamp01(
            (float)(elapsed / duration));

        if (elapsed <= duration)
        {
            Vector3 nominalPosition =
                EvaluateNormalTrajectory(state, phase);

            Vector3 newPredictedPosition =
                ApplyAnchorCorrection(
                    leg,
                    state,
                    nominalPosition,
                    now);

            target.position =
                ApplyPredictionBlend(
                    leg,
                    state,
                    newPredictedPosition,
                    now);

            if (!state.LandingEventFired &&
                phase >= groundStartPhase)
            {
                state.LandingEventFired = true;

                RaiseLandingEvent(
                    leg,
                    state.LandingPosition,
                    now,
                    LandingReason.Predicted);
            }

            return;
        }

        SetAnchorBlendActive(leg, false);
        state.AnchorCorrectionActive = false;

        Vector3 completedPrediction =
            ApplyPredictionBlend(
                leg,
                state,
                state.NextPeakPosition,
                now);

        target.position = completedPrediction;
        phase = 1f;

        if (elapsed >= duration + eventTimeoutGrace)
        {
            StartFallbackLanding(
                leg,
                state,
                target,
                groundY,
                now);
        }
    }

    // ===== Post-cycle Avatar Follow =====
    // 仅在双腿完全静止且没有全局运动时，使站立 Foot Target 保持在 Avatar 下方。
    private bool ShouldPostTrajectoryAvatarFollow()
    {
        return calibrated &&
               peakAnchorTransform != null &&
               leftFootTarget != null &&
               rightFootTarget != null &&
               (leftState.HasPreviousEvent ||
                rightState.HasPreviousEvent) &&
               AreBothLegsQuiescent();
    }

    private bool AreBothLegsQuiescent()
    {
        return !leftState.IsActive &&
               !rightState.IsActive &&
               !leftState.FallbackLandingActive &&
               !rightState.FallbackLandingActive &&
               !leftState.AnchorCorrectionActive &&
               !rightState.AnchorCorrectionActive &&
               !leftState.PredictionBlendActive &&
               !rightState.PredictionBlendActive;
    }

    private void UpdateAvatarRelativeStandingFoot(
        PeakLegSide leg,
        Transform target,
        float groundY)
    {
        if (!calibrated ||
            target == null ||
            peakAnchorTransform == null)
        {
            return;
        }

        Vector3 localHorizontalOffset =
            leg == PeakLegSide.Left
                ? leftLocalHorizontalOffset
                : rightLocalHorizontalOffset;

        Vector3 standingPosition =
            peakAnchorTransform.TransformPoint(
                localHorizontalOffset);

        standingPosition.y = groundY;
        target.position = standingPosition;
    }

    // ===== 兼容单腿 Normal Trajectory =====
    // 计算 Current Peak -> Landing -> Next Peak 的分段轨迹。
    // 虽然正常步态主要由全局 T/4 生成，此路径仍与兼容/恢复状态共享，不能随意删除。
    private Vector3 EvaluateNormalTrajectory(
        LegState state,
        float phase)
    {
        if (phase <= groundStartPhase)
        {
            float localT =
                Mathf.InverseLerp(
                    0f,
                    groundStartPhase,
                    phase);

            return Vector3.Lerp(
                state.CurrentPeakPosition,
                state.LandingPosition,
                EvaluateTrajectoryCurve(localT));
        }

        if (phase <= groundEndPhase)
        {
            // World-space stance lock.
            return state.LandingPosition;
        }

        float riseT =
            Mathf.InverseLerp(
                groundEndPhase,
                1f,
                phase);

        return Vector3.Lerp(
            state.LandingPosition,
            state.NextPeakPosition,
            EvaluateTrajectoryCurve(riseT));
    }

    // ===== Anchor Correction =====
    // 当 Avatar-relative 新 Peak 与旧 Foot Target 有偏差时，短时消除位置跳变。
    private Vector3 ApplyAnchorCorrection(
        PeakLegSide leg,
        LegState state,
        Vector3 nominalPosition,
        double now)
    {
        if (!state.AnchorCorrectionActive)
        {
            SetAnchorBlendActive(leg, false);
            return nominalPosition;
        }

        float duration =
            Mathf.Max(0.0001f, anchorCorrectionDuration);

        float progress =
            Mathf.Clamp01(
                (float)((now -
                         state.AnchorCorrectionStartTime) / duration));

        float blend =
            EvaluateAnchorCurve(progress);

        Vector3 correctedPosition =
            nominalPosition +
            state.AnchorCorrectionOffset * (1f - blend);

        if (progress >= 1f)
        {
            state.AnchorCorrectionActive = false;
            SetAnchorBlendActive(leg, false);
        }

        return correctedPosition;
    }

    private PredictionSnapshot CapturePredictionSnapshot(
        LegState state,
        Vector3 currentTargetPosition,
        double now)
    {
        PredictionSnapshot snapshot =
            new PredictionSnapshot
            {
                IsValid = true,
                WasActive = state.IsActive,
                WasFallbackLandingActive =
                    state.FallbackLandingActive,
                CycleStartUnityTime =
                    state.CycleStartUnityTime,
                CycleDuration =
                    state.CycleDuration,
                CurrentPeakPosition =
                    state.CurrentPeakPosition,
                LandingPosition =
                    state.LandingPosition,
                NextPeakPosition =
                    state.NextPeakPosition,
                FallbackLandingStartTime =
                    state.FallbackLandingStartTime,
                FallbackStartPosition =
                    state.FallbackStartPosition,
                FallbackGroundPosition =
                    state.FallbackGroundPosition,
                StaticPosition =
                    currentTargetPosition
            };

        Vector3 predictedAtCapture =
            EvaluatePredictionSnapshot(snapshot, now);

        snapshot.ContinuityOffset =
            currentTargetPosition - predictedAtCapture;

        return snapshot;
    }

    // ===== Prediction-to-Prediction Blend =====
    // 后续 Peak 更新预测时，在旧预测轨迹与新预测轨迹之间短时插值，保持连续。
    private Vector3 ApplyPredictionBlend(
        PeakLegSide leg,
        LegState state,
        Vector3 newPredictedPosition,
        double now)
    {
        if (!state.PredictionBlendActive ||
            state.PreviousPrediction == null ||
            !state.PreviousPrediction.IsValid)
        {
            SetPredictionBlendActive(leg, false, 1f);
            return newPredictedPosition;
        }

        float duration =
            Mathf.Max(0.0001f, predictionBlendDuration);

        float progress =
            Mathf.Clamp01(
                (float)((now -
                         state.PredictionBlendStartTime) /
                        duration));

        float blend =
            EvaluatePredictionBlendCurve(progress);

        Vector3 oldPredictedPosition =
            EvaluatePredictionSnapshot(
                state.PreviousPrediction,
                now);

        Vector3 blendedPosition =
            Vector3.Lerp(
                oldPredictedPosition,
                newPredictedPosition,
                blend);

        SetPredictionBlendActive(
            leg,
            progress < 1f,
            progress);

        if (progress >= 1f)
        {
            state.PredictionBlendActive = false;
            state.PreviousPrediction = null;
        }

        return blendedPosition;
    }

    private Vector3 EvaluatePredictionSnapshot(
        PredictionSnapshot snapshot,
        double now)
    {
        if (snapshot == null || !snapshot.IsValid)
        {
            return Vector3.zero;
        }

        if (snapshot.WasFallbackLandingActive)
        {
            float fallbackDuration =
                Mathf.Max(0.05f, fallbackLandingDuration);

            float fallbackT =
                Mathf.Clamp01(
                    (float)((now -
                             snapshot.FallbackLandingStartTime) /
                            fallbackDuration));

            return Vector3.Lerp(
                       snapshot.FallbackStartPosition,
                       snapshot.FallbackGroundPosition,
                       EvaluateTrajectoryCurve(fallbackT)) +
                   snapshot.ContinuityOffset;
        }

        if (!snapshot.WasActive)
        {
            return snapshot.StaticPosition;
        }

        float duration =
            Mathf.Max(0.001f, snapshot.CycleDuration);

        float phase =
            Mathf.Clamp01(
                (float)((now -
                         snapshot.CycleStartUnityTime) /
                        duration));

        Vector3 position =
            EvaluateTrajectorySnapshot(snapshot, phase);

        return position + snapshot.ContinuityOffset;
    }

    private Vector3 EvaluateTrajectorySnapshot(
        PredictionSnapshot snapshot,
        float phase)
    {
        if (phase <= groundStartPhase)
        {
            float localT =
                Mathf.InverseLerp(
                    0f,
                    groundStartPhase,
                    phase);

            return Vector3.Lerp(
                snapshot.CurrentPeakPosition,
                snapshot.LandingPosition,
                EvaluateTrajectoryCurve(localT));
        }

        if (phase <= groundEndPhase)
        {
            return snapshot.LandingPosition;
        }

        float riseT =
            Mathf.InverseLerp(
                groundEndPhase,
                1f,
                phase);

        return Vector3.Lerp(
            snapshot.LandingPosition,
            snapshot.NextPeakPosition,
            EvaluateTrajectoryCurve(riseT));
    }

    // ===== Missing-event Fallback Landing =====
    // 超过事件等待时间后，从当前 Foot Target 位置平滑返回固定 Ground Y。
    private void StartFallbackLanding(
        PeakLegSide leg,
        LegState state,
        Transform target,
        float groundY,
        double now)
    {
        state.AnchorCorrectionActive = false;
        SetAnchorBlendActive(leg, false);

        state.PredictionBlendActive = false;
        state.PreviousPrediction = null;
        SetPredictionBlendActive(leg, false, 1f);

        state.FallbackLandingActive = true;
        state.FallbackLandingStartTime = now;
        state.FallbackStartPosition = target.position;

        state.FallbackGroundPosition =
            new Vector3(
                target.position.x,
                groundY,
                target.position.z);

        SetFallbackActive(leg, true);
        IncrementFallbackCount(leg);

        if (logFallbackLanding)
        {
            Debug.Log(
                $"[PeakTrajectorySmooth] {leg} event timeout. " +
                "Returning foot to ground.",
                this);
        }
    }

    private void UpdateFallbackLanding(
        PeakLegSide leg,
        LegState state,
        Transform target,
        double now)
    {
        float duration =
            Mathf.Max(0.05f, fallbackLandingDuration);

        float t =
            Mathf.Clamp01(
                (float)((now -
                         state.FallbackLandingStartTime) / duration));

        target.position =
            Vector3.Lerp(
                state.FallbackStartPosition,
                state.FallbackGroundPosition,
                EvaluateTrajectoryCurve(t));

        if (t < 1f)
        {
            return;
        }

        target.position = state.FallbackGroundPosition;

        state.FallbackLandingActive = false;
        state.IsActive = false;

        SetFallbackActive(leg, false);
    }

    // ===== 对外 Landing 事件 =====
    // 更新诊断计数并触发 FootLanded；CSV 与 Ripple 等观察者由此获知落地。
    private void RaiseLandingEvent(
        PeakLegSide leg,
        Vector3 position,
        double unityTime,
        LandingReason reason)
    {
        if (leg == PeakLegSide.Left)
        {
            leftPredictedLandingCount++;
            lastLeftLandingTime = unityTime;
            lastLeftLandingPosition = position;
        }
        else
        {
            rightPredictedLandingCount++;
            lastRightLandingTime = unityTime;
            lastRightLandingPosition = position;
        }

        FootLandingEvent landingEvent =
            new FootLandingEvent(
                leg,
                unityTime,
                position,
                reason);

        FootLanded?.Invoke(landingEvent);

        if (logLandingEvents)
        {
            Debug.Log(
                $"[PeakTrajectorySmooth] landing leg={leg}, " +
                $"reason={reason}, " +
                $"time={unityTime:F3}, " +
                $"position={position}",
                this);
        }
    }

    // ===== 对侧腿支撑保护 =====
    // 新 Peak 被接受时，必要时把对侧 Foot Target 立即放回 Ground Y，
    // 防止两条腿同时离地。
    private void EnforceOppositeLegSupport(
        PeakLegSide eventLeg,
        double now)
    {
        PeakLegSide supportLeg =
            eventLeg == PeakLegSide.Left
                ? PeakLegSide.Right
                : PeakLegSide.Left;

        LegState supportState =
            supportLeg == PeakLegSide.Left
                ? leftState
                : rightState;

        Transform supportTarget =
            supportLeg == PeakLegSide.Left
                ? leftFootTarget
                : rightFootTarget;

        float supportGroundY =
            supportLeg == PeakLegSide.Left
                ? leftGroundY
                : rightGroundY;

        if (supportTarget == null)
        {
            return;
        }

        bool wasAboveGround =
            Mathf.Abs(
                supportTarget.position.y -
                supportGroundY) >
            directCorrectionThreshold;

        bool wasMoving =
            supportState.IsActive ||
            supportState.FallbackLandingActive ||
            supportState.AnchorCorrectionActive;

        if (!wasAboveGround && !wasMoving)
        {
            return;
        }

        // Keep the current X/Z location and place the support foot on
        // the ground immediately. This guarantees that the newly
        // detected peak does not overlap with an airborne opposite foot.
        Vector3 supportPosition =
            supportTarget.position;

        supportPosition.y = supportGroundY;
        supportTarget.position = supportPosition;

        supportState.IsActive = false;
        supportState.AnchorCorrectionActive = false;
        supportState.PredictionBlendActive = false;
        supportState.PreviousPrediction = null;
        supportState.FallbackLandingActive = false;

        supportState.LandingPosition = supportPosition;
        supportState.NextPeakPosition = supportPosition;
        supportState.FallbackGroundPosition = supportPosition;
        supportState.FallbackStartPosition = supportPosition;
        supportState.FallbackLandingStartTime = now;

        SetAnchorBlendActive(supportLeg, false);
        SetPredictionBlendActive(supportLeg, false, 1f);
        SetFallbackActive(supportLeg, false);
        IncrementForcedSupportCount(supportLeg);

        if (logOppositeSupport)
        {
            Debug.Log(
                $"[PeakTrajectorySmooth] {eventLeg} peak received. " +
                $"{supportLeg} foot forced to ground support.",
                this);
        }
    }

    private void IncrementForcedSupportCount(
        PeakLegSide supportLeg)
    {
        if (supportLeg == PeakLegSide.Left)
        {
            leftForcedSupportCount++;
        }
        else
        {
            rightForcedSupportCount++;
        }
    }

    // ===== Avatar-relative Peak 构造 =====
    // X/Z 来自初始脚偏移相对 Avatar Root 的变换，Y 固定为 Ground Y + Peak Height。
    private Vector3 BuildAnchoredPeakPosition(
        PeakLegSide leg,
        Vector3 fallbackPosition,
        float groundY)
    {
        Vector3 position = fallbackPosition;

        if (peakAnchorTransform != null)
        {
            if (keepInitialFootOffsets)
            {
                Vector3 offset =
                    leg == PeakLegSide.Left
                        ? leftLocalHorizontalOffset
                        : rightLocalHorizontalOffset;

                position =
                    peakAnchorTransform.TransformPoint(offset);
            }
            else
            {
                position = peakAnchorTransform.position;
            }
        }

        position.y = groundY + peakHeight;
        return position;
    }

    // ===== HMD 位移 -> 预测步长 =====
    // 只使用水平位移，经死区、比例和最大距离 Clamp 后得到 raw step distance。
    private float CalculatePredictedDistance(
        Vector3 horizontalDelta)
    {
        float measuredDistance =
            horizontalDelta.magnitude;

        if (measuredDistance <
            minimumReferenceDisplacement)
        {
            return 0f;
        }

        return Mathf.Clamp(
            measuredDistance *
            Mathf.Max(0f, displacementScale),
            0f,
            maximumPredictedDistance);
    }


    // ===== 步长指数平滑 =====
    // 对 raw step distance 做跨事件低通滤波，减少追踪噪声造成的落点跳动。
    private float ApplyStepLengthSmoothing(
        LegState state,
        float rawDistance,
        bool hasValidSameLegReference)
    {
        if (!hasValidSameLegReference)
        {
            return 0f;
        }

        if (!state.HasFilteredStepLength)
        {
            state.FilteredStepLength =
                initialiseFilterFromFirstEstimate
                    ? rawDistance
                    : rawDistance *
                      (1f - Mathf.Clamp01(stepLengthSmoothingAlpha));

            state.HasFilteredStepLength = true;
            return state.FilteredStepLength;
        }

        float alpha =
            Mathf.Clamp(stepLengthSmoothingAlpha, 0f, 0.99f);

        state.FilteredStepLength =
            state.FilteredStepLength * alpha +
            rawDistance * (1f - alpha);

        state.FilteredStepLength =
            Mathf.Clamp(
                state.FilteredStepLength,
                0f,
                maximumPredictedDistance);

        return state.FilteredStepLength;
    }

    private Vector3 GetHorizontalDirection(
        Vector3 horizontalDelta)
    {
        if (horizontalDelta.sqrMagnitude >
            0.000001f)
        {
            return horizontalDelta.normalized;
        }

        Vector3 direction =
            directionTransform != null
                ? directionTransform.forward
                : Vector3.forward;

        direction.y = 0f;

        return direction.sqrMagnitude < 0.000001f
            ? Vector3.forward
            : direction.normalized;
    }

    private float EvaluateTrajectoryCurve(float value)
    {
        return trajectorySmoothingCurve == null
            ? value
            : trajectorySmoothingCurve.Evaluate(value);
    }

    private float EvaluateAnchorCurve(float value)
    {
        return anchorCorrectionCurve == null
            ? value
            : anchorCorrectionCurve.Evaluate(value);
    }

    private float EvaluatePredictionBlendCurve(float value)
    {
        return predictionBlendCurve == null
            ? value
            : predictionBlendCurve.Evaluate(value);
    }

    private void SetPredictionBlendActive(
        PeakLegSide leg,
        bool active,
        float progress)
    {
        if (leg == PeakLegSide.Left)
        {
            leftPredictionBlendActive = active;
            leftPredictionBlendProgress = progress;
        }
        else
        {
            rightPredictionBlendActive = active;
            rightPredictionBlendProgress = progress;
        }
    }

    private void SetAnchorBlendActive(
        PeakLegSide leg,
        bool active)
    {
        if (leg == PeakLegSide.Left)
        {
            leftAnchorBlendActive = active;
        }
        else
        {
            rightAnchorBlendActive = active;
        }
    }

    private void SetFallbackActive(
        PeakLegSide leg,
        bool active)
    {
        if (leg == PeakLegSide.Left)
        {
            leftFallbackLandingActive = active;
        }
        else
        {
            rightFallbackLandingActive = active;
        }
    }

    private void IncrementFallbackCount(
        PeakLegSide leg)
    {
        if (leg == PeakLegSide.Left)
        {
            leftFallbackLandingCount++;
        }
        else
        {
            rightFallbackLandingCount++;
        }
    }

    // ===== 生命周期收尾 =====
    // 清空 Pending 并解除 PeakApplied 订阅，防止组件再次启用时重复接收事件。
    private void OnDisable()
    {
        ClearPendingExpectedPeak("OnDisable", false);

        if (receiver != null)
        {
            receiver.PeakApplied -= HandlePeakApplied;
        }
    }

    public enum LandingReason
    {
        Predicted
    }

    private enum GaitPhase
    {
        StartupIdle,
        LeftLowering,
        RightRising,
        RightPeakHold,
        RightLowering,
        LeftRising,
        LeftPeakHold,
        Recovery,
        AvatarFollowIdle
    }

    private enum PeakEventDisposition
    {
        AcceptNow,
        CacheExpectedEarly,
        RejectOrIgnore
    }

    public readonly struct FootLandingEvent
    {
        public FootLandingEvent(
            PeakLegSide leg,
            double unityTimestamp,
            Vector3 worldPosition,
            LandingReason reason)
        {
            Leg = leg;
            UnityTimestamp = unityTimestamp;
            WorldPosition = worldPosition;
            Reason = reason;
        }

        public PeakLegSide Leg { get; }
        public double UnityTimestamp { get; }
        public Vector3 WorldPosition { get; }
        public LandingReason Reason { get; }
    }

    [Serializable]
    private sealed class PredictionSnapshot
    {
        public bool IsValid;
        public bool WasActive;
        public bool WasFallbackLandingActive;

        public double CycleStartUnityTime;
        public float CycleDuration;

        public Vector3 CurrentPeakPosition;
        public Vector3 LandingPosition;
        public Vector3 NextPeakPosition;

        public double FallbackLandingStartTime;
        public Vector3 FallbackStartPosition;
        public Vector3 FallbackGroundPosition;

        public Vector3 StaticPosition;
        public Vector3 ContinuityOffset;
    }

    [Serializable]
    private sealed class LegState //单条腿的运行状态记录表
    {
        public bool IsActive;

        public bool HasPreviousEvent;
        public double PreviousUnityEventTime;

        public bool HasPreviousReference;
        public Vector3 PreviousReferencePosition;

        public bool HasFilteredStepLength;
        public float FilteredStepLength;

        public double CycleStartUnityTime;
        public float CycleDuration;

        public Vector3 CurrentPeakPosition;
        public Vector3 LandingPosition;
        public Vector3 NextPeakPosition;

        public bool LandingEventFired;

        public bool AnchorCorrectionActive;
        public double AnchorCorrectionStartTime;
        public Vector3 AnchorCorrectionOffset;

        public bool PredictionBlendActive;
        public double PredictionBlendStartTime;
        public PredictionSnapshot PreviousPrediction;

        public bool FallbackLandingActive;
        public double FallbackLandingStartTime;
        public Vector3 FallbackStartPosition;
        public Vector3 FallbackGroundPosition;
    }
}
