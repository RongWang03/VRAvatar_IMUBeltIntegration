using UnityEngine;

[DefaultExecutionOrder(-100)]
public sealed class FirstPersonAvatarEyeAligner : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Transform hmdTransform;
    [SerializeField] private Transform leftEye;
    [SerializeField] private Transform rightEye;

    [Header("Horizontal Alignment")]
    [Min(0f)]
    [SerializeField] private float positionSmoothing = 12f;

    [Header("Diagnostics")]
    [SerializeField] private Vector3 eyeCenter;
    [SerializeField] private Vector2 horizontalError;
    [SerializeField] private Vector3 targetRootPosition;

    private float fixedRootY;
    private bool initialized;

    private void Start()
    {
        fixedRootY = transform.position.y;
        initialized = ValidateReferences();

        if (!initialized)
        {
            Debug.LogWarning(
                "[FirstPersonAvatarEyeAligner] HMD and both eye references must be assigned.",
                this);
            return;
        }

        eyeCenter = CalculateEyeCenter();
        horizontalError = new Vector2(
            hmdTransform.position.x - eyeCenter.x,
            hmdTransform.position.z - eyeCenter.z);
    }

    private void Update()
    {
        if (!initialized)
        {
            return;
        }

        eyeCenter = CalculateEyeCenter();
        horizontalError = new Vector2(
            hmdTransform.position.x - eyeCenter.x,
            hmdTransform.position.z - eyeCenter.z);

        Vector3 rootPosition = transform.position;
        targetRootPosition = new Vector3(
            rootPosition.x + horizontalError.x,
            fixedRootY,
            rootPosition.z + horizontalError.y);

        if (positionSmoothing <= 0f)
        {
            transform.position = targetRootPosition;
            return;
        }

        float t = 1f - Mathf.Exp(-positionSmoothing * Time.deltaTime);
        Vector3 smoothedPosition = Vector3.Lerp(rootPosition, targetRootPosition, t);
        smoothedPosition.y = fixedRootY;
        transform.position = smoothedPosition;
    }

    private bool ValidateReferences()
    {
        return hmdTransform != null &&
               leftEye != null &&
               rightEye != null;
    }

    private Vector3 CalculateEyeCenter()
    {
        return (leftEye.position + rightEye.position) * 0.5f;
    }
}
