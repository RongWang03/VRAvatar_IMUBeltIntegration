using UnityEngine;

[DefaultExecutionOrder(-10000)]
public sealed class StartupAvatarHeightCalibrationCoordinator : MonoBehaviour
{
    [Header("Calibration References")]
    [SerializeField] private Transform avatarRoot;
    [SerializeField] private Transform hmdTransform;
    [SerializeField] private Transform leftEye;
    [SerializeField] private Transform rightEye;
    [SerializeField] private FirstPersonAvatarEyeAligner eyeAligner;
    [SerializeField] private GameObject lowerBodyControllerObject;

    [Header("Static Geometry Diagnostics")]
    [SerializeField] private Transform hips;
    [SerializeField] private Transform leftUpperLeg;
    [SerializeField] private Transform leftLowerLeg;
    [SerializeField] private Transform leftFoot;
    [SerializeField] private Transform rightUpperLeg;
    [SerializeField] private Transform rightLowerLeg;
    [SerializeField] private Transform rightFoot;
    [SerializeField] private Transform leftFootTarget;
    [SerializeField] private Transform rightFootTarget;

    [Header("Startup Timing")]
    [Min(0f)]
    [SerializeField] private float stableTrackingSeconds = 2f;
    [Min(0f)]
    [SerializeField] private float minimumValidHmdHeight = 0.2f;
    [Min(0f)]
    [SerializeField] private float maximumFrameHeightJump = 0.1f;

    [Header("Calibration Diagnostics")]
    [SerializeField] private bool trackingValid;
    [SerializeField] private float stableTrackingElapsed;
    [SerializeField] private bool calibrationApplied;
    [SerializeField] private float hmdEyeYBefore;
    [SerializeField] private float avatarEyeCenterYBefore;
    [SerializeField] private float eyeVerticalErrorBefore;
    [SerializeField] private float appliedVerticalOffset;
    [SerializeField] private float calibratedRootY;
    [SerializeField] private float hmdEyeYAfter;
    [SerializeField] private float avatarEyeCenterYAfter;
    [SerializeField] private float eyeVerticalErrorAfter;

    [Header("Post-Calibration Leg Diagnostics")]
    [SerializeField] private bool staticGeometrySampled;
    [SerializeField] private bool staticGeometryReasonable;
    [SerializeField] private float hipsY;
    [SerializeField] private float leftFootBoneY;
    [SerializeField] private float rightFootBoneY;
    [SerializeField] private float leftFootTargetY;
    [SerializeField] private float rightFootTargetY;
    [SerializeField] private float leftHipsToTargetDistance;
    [SerializeField] private float rightHipsToTargetDistance;
    [SerializeField] private float leftLegChainLength;
    [SerializeField] private float rightLegChainLength;
    [SerializeField] private float leftReachRatio;
    [SerializeField] private float rightReachRatio;
    [SerializeField] private float leftKneeAngle;
    [SerializeField] private float rightKneeAngle;

    private float lastValidHmdY;
    private bool hasLastValidHmdY;
    private int framesAfterActivation;

    private void Awake()
    {
        if (lowerBodyControllerObject != null)
        {
            lowerBodyControllerObject.SetActive(false);
        }

        if (eyeAligner != null)
        {
            eyeAligner.enabled = false;
        }
    }

    private void Update()
    {
        if (!calibrationApplied)
        {
            UpdateTrackingAndCalibrate();
            return;
        }

        if (!staticGeometrySampled)
        {
            framesAfterActivation++;
            if (framesAfterActivation >= 2)
            {
                SampleStaticGeometry();
            }
        }
    }

    private void UpdateTrackingAndCalibrate()
    {
        if (!ValidateCalibrationReferences())
        {
            trackingValid = false;
            return;
        }

        float currentHmdY = hmdTransform.position.y;
        trackingValid = IsFinite(currentHmdY) && currentHmdY > minimumValidHmdHeight;

        if (!trackingValid)
        {
            stableTrackingElapsed = 0f;
            hasLastValidHmdY = false;
            return;
        }

        if (hasLastValidHmdY &&
            Mathf.Abs(currentHmdY - lastValidHmdY) > maximumFrameHeightJump)
        {
            stableTrackingElapsed = 0f;
        }
        else
        {
            stableTrackingElapsed += Time.unscaledDeltaTime;
        }

        lastValidHmdY = currentHmdY;
        hasLastValidHmdY = true;

        if (stableTrackingElapsed >= stableTrackingSeconds)
        {
            ApplyOneTimeCalibration();
        }
    }

private void ApplyOneTimeCalibration()
    {
        hmdEyeYBefore = hmdTransform.position.y;
        avatarEyeCenterYBefore = CalculateEyeCenter().y;
        eyeVerticalErrorBefore = hmdEyeYBefore - avatarEyeCenterYBefore;
        appliedVerticalOffset = eyeVerticalErrorBefore;

        Vector3 rootPosition = avatarRoot.position;
        rootPosition.y += appliedVerticalOffset;
        avatarRoot.position = rootPosition;
        calibratedRootY = avatarRoot.position.y;

        hmdEyeYAfter = hmdTransform.position.y;
        avatarEyeCenterYAfter = CalculateEyeCenter().y;
        eyeVerticalErrorAfter = hmdEyeYAfter - avatarEyeCenterYAfter;
        calibrationApplied = true;

        if (eyeAligner != null)
        {
            eyeAligner.enabled = true;
        }

        if (lowerBodyControllerObject != null)
        {
            lowerBodyControllerObject.SetActive(true);
        }

        Debug.LogFormat(
            this,
            "[HeightCalibration] Applied once. BeforeError={0:F4} m, RootOffset={1:F4} m, AfterError={2:F4} m. Lower body activated after calibration.",
            eyeVerticalErrorBefore,
            appliedVerticalOffset,
            eyeVerticalErrorAfter);
    }

private void SampleStaticGeometry()
    {
        if (!ValidateGeometryReferences())
        {
            Debug.LogWarning(
                "[HeightCalibration] Geometry diagnostics incomplete because one or more leg references are missing.",
                this);
            staticGeometrySampled = true;
            staticGeometryReasonable = false;
            return;
        }

        hipsY = hips.position.y;
        leftFootBoneY = leftFoot.position.y;
        rightFootBoneY = rightFoot.position.y;
        leftFootTargetY = leftFootTarget.position.y;
        rightFootTargetY = rightFootTarget.position.y;

        leftHipsToTargetDistance = Vector3.Distance(hips.position, leftFootTarget.position);
        rightHipsToTargetDistance = Vector3.Distance(hips.position, rightFootTarget.position);
        leftLegChainLength =
            Vector3.Distance(leftUpperLeg.position, leftLowerLeg.position) +
            Vector3.Distance(leftLowerLeg.position, leftFoot.position);
        rightLegChainLength =
            Vector3.Distance(rightUpperLeg.position, rightLowerLeg.position) +
            Vector3.Distance(rightLowerLeg.position, rightFoot.position);

        leftReachRatio = SafeRatio(leftHipsToTargetDistance, leftLegChainLength);
        rightReachRatio = SafeRatio(rightHipsToTargetDistance, rightLegChainLength);
        leftKneeAngle = CalculateJointAngle(leftUpperLeg.position, leftLowerLeg.position, leftFoot.position);
        rightKneeAngle = CalculateJointAngle(rightUpperLeg.position, rightLowerLeg.position, rightFoot.position);

        staticGeometryReasonable =
            leftReachRatio <= 1.02f &&
            rightReachRatio <= 1.02f &&
            leftReachRatio >= 0.45f &&
            rightReachRatio >= 0.45f;

        staticGeometrySampled = true;

        const string format = "[HeightCalibration] Static geometry: RootY={0:F4}, HipsY={1:F4}, LeftFoot/TargetY={2:F4}/{3:F4}, RightFoot/TargetY={4:F4}/{5:F4}, ReachRatio L/R={6:F3}/{7:F3}, KneeAngle L/R={8:F1}/{9:F1}, Reasonable={10}. Do not send Peak until the standing pose is visually accepted.";

        object[] values =
        {
            calibratedRootY,
            hipsY,
            leftFootBoneY,
            leftFootTargetY,
            rightFootBoneY,
            rightFootTargetY,
            leftReachRatio,
            rightReachRatio,
            leftKneeAngle,
            rightKneeAngle,
            staticGeometryReasonable
        };

        if (staticGeometryReasonable)
        {
            Debug.LogFormat(this, format, values);
        }
        else
        {
            Debug.LogWarningFormat(this, format, values);
        }
    }

    private Vector3 CalculateEyeCenter()
    {
        return (leftEye.position + rightEye.position) * 0.5f;
    }

    private bool ValidateCalibrationReferences()
    {
        return avatarRoot != null &&
               hmdTransform != null &&
               leftEye != null &&
               rightEye != null &&
               lowerBodyControllerObject != null;
    }

    private bool ValidateGeometryReferences()
    {
        return hips != null &&
               leftUpperLeg != null &&
               leftLowerLeg != null &&
               leftFoot != null &&
               rightUpperLeg != null &&
               rightLowerLeg != null &&
               rightFoot != null &&
               leftFootTarget != null &&
               rightFootTarget != null;
    }

    private static float CalculateJointAngle(Vector3 upper, Vector3 joint, Vector3 lower)
    {
        return Vector3.Angle(upper - joint, lower - joint);
    }

    private static float SafeRatio(float numerator, float denominator)
    {
        return denominator > 0.0001f ? numerator / denominator : float.PositiveInfinity;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
