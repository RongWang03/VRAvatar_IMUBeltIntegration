using UnityEngine;

[DefaultExecutionOrder(22000)]
public sealed class FirstPersonEyeVerticalDiagnostics : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform hmdTransform;
    [SerializeField] private Transform leftEye;
    [SerializeField] private Transform rightEye;

    [Header("Diagnostic Timing")]
    [Min(0f)]
    [SerializeField] private float stableSampleDelay = 2f;

    [Header("Runtime Read-Only Diagnostics")]
    [SerializeField] private float hmdEyeY;
    [SerializeField] private float avatarEyeCenterY;
    [SerializeField] private float eyeVerticalError;
    [SerializeField] private bool hasValidTrackingSample;
    [SerializeField] private bool stableSampleLogged;

    public float HmdEyeY => hmdEyeY;
    public float AvatarEyeCenterY => avatarEyeCenterY;
    public float EyeVerticalError => eyeVerticalError;
    public bool HasValidTrackingSample => hasValidTrackingSample;

    private void OnEnable()
    {
        stableSampleLogged = false;
        hasValidTrackingSample = false;
    }

    private void LateUpdate()
    {
        if (hmdTransform == null || leftEye == null || rightEye == null)
        {
            hasValidTrackingSample = false;
            return;
        }

        hmdEyeY = hmdTransform.position.y;
        avatarEyeCenterY = (leftEye.position.y + rightEye.position.y) * 0.5f;
        eyeVerticalError = hmdEyeY - avatarEyeCenterY;

        hasValidTrackingSample =
            Time.unscaledTime >= stableSampleDelay &&
            Mathf.Abs(hmdEyeY) > 0.2f;

        if (!hasValidTrackingSample || stableSampleLogged)
        {
            return;
        }

        stableSampleLogged = true;
        Debug.Log(
            $"[FirstPersonEyeVertical] HmdEyeY={hmdEyeY:F4}, " +
            $"AvatarEyeCenterY={avatarEyeCenterY:F4}, " +
            $"EyeVerticalError={eyeVerticalError:F4}",
            this);
    }
}
