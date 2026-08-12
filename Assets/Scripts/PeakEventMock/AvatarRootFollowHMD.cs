using UnityEngine;

/// <summary>
/// Keeps the avatar root horizontally aligned with the VR headset.
/// The avatar Y position stays unchanged.
/// Foot targets can remain in world space during stance.
/// </summary>
[DefaultExecutionOrder(-100)]
public sealed class AvatarRootFollowHMD : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform hmdTransform;

    [Header("Position Follow")]
    [SerializeField] private bool keepInitialHorizontalOffset = true;

    [Tooltip("0 means direct follow. Higher values add smoothing.")]
    [Min(0f)]
    [SerializeField] private float positionSmoothing = 0f;

    [Header("Optional Rotation")]
    [SerializeField] private bool followHeadsetYaw = false;

    [Min(0f)]
    [SerializeField] private float yawSmoothing = 8f;

    [Header("Diagnostics")]
    [SerializeField] private Vector3 initialHorizontalOffset;
    [SerializeField] private Vector3 targetRootPosition;
    [SerializeField] private float targetYaw;

    private float fixedRootY;
    private bool calibrated;

    private void Start()
    {
        Calibrate();
    }

    [ContextMenu("Calibrate")]
    public void Calibrate()
    {
        fixedRootY = transform.position.y;

        if (hmdTransform == null)
        {
            Debug.LogWarning(
                "[AvatarRootFollowHMD] HMD Transform is not assigned.",
                this);

            calibrated = false;
            return;
        }

        if (keepInitialHorizontalOffset)
        {
            initialHorizontalOffset = new Vector3(
                transform.position.x - hmdTransform.position.x,
                0f,
                transform.position.z - hmdTransform.position.z);
        }
        else
        {
            initialHorizontalOffset = Vector3.zero;
        }

        calibrated = true;
    }

    private void Update()
    {
        if (!calibrated || hmdTransform == null)
        {
            return;
        }

        targetRootPosition = new Vector3(
            hmdTransform.position.x + initialHorizontalOffset.x,
            fixedRootY,
            hmdTransform.position.z + initialHorizontalOffset.z);

        if (positionSmoothing <= 0f)
        {
            transform.position = targetRootPosition;
        }
        else
        {
            float t = 1f - Mathf.Exp(
                -positionSmoothing * Time.deltaTime);

            transform.position = Vector3.Lerp(
                transform.position,
                targetRootPosition,
                t);
        }

        if (!followHeadsetYaw)
        {
            return;
        }

        Vector3 flatForward = hmdTransform.forward;
        flatForward.y = 0f;

        if (flatForward.sqrMagnitude < 0.000001f)
        {
            return;
        }

        targetYaw = Quaternion.LookRotation(
            flatForward.normalized,
            Vector3.up).eulerAngles.y;

        Quaternion targetRotation =
            Quaternion.Euler(0f, targetYaw, 0f);

        if (yawSmoothing <= 0f)
        {
            transform.rotation = targetRotation;
        }
        else
        {
            float t = 1f - Mathf.Exp(
                -yawSmoothing * Time.deltaTime);

            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                targetRotation,
                t);
        }
    }
}
