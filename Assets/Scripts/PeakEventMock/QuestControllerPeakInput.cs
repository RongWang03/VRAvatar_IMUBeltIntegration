using UnityEngine;

/// <summary>
/// Adapts Quest Touch controller face-button presses to the shared
/// LegPeakEventReceiver input path. This component never writes foot targets.
/// </summary>
public sealed class QuestControllerPeakInput : MonoBehaviour
{
    [Header("Output")]
    [SerializeField] private LegPeakEventReceiver peakEventReceiver;

    [Header("Debounce")]
    [Min(0f)]
    [SerializeField] private float cooldownSeconds = 0.12f;

    [Header("Diagnostics")]
    [SerializeField] private int leftPeakSubmitCount;
    [SerializeField] private int rightPeakSubmitCount;
    [SerializeField] private double lastLeftPeakUnityTime;
    [SerializeField] private double lastRightPeakUnityTime;

    private double nextLeftAllowedTime;
    private double nextRightAllowedTime;

    public int LeftPeakSubmitCount => leftPeakSubmitCount;
    public int RightPeakSubmitCount => rightPeakSubmitCount;
    public double LastLeftPeakUnityTime => lastLeftPeakUnityTime;
    public double LastRightPeakUnityTime => lastRightPeakUnityTime;

    private void Awake()
    {
        if (peakEventReceiver == null)
        {
            peakEventReceiver =
                FindFirstObjectByType<LegPeakEventReceiver>();
        }
    }

    private void Update()
    {
        double now = Time.realtimeSinceStartupAsDouble;

        if (OVRInput.GetDown(
                OVRInput.RawButton.Y,
                OVRInput.Controller.LTouch))
        {
            TrySubmitPeak(
                PeakLegSide.Left,
                now);
        }

        if (OVRInput.GetDown(
                OVRInput.RawButton.B,
                OVRInput.Controller.RTouch))
        {
            TrySubmitPeak(
                PeakLegSide.Right,
                now);
        }
    }

    private void TrySubmitPeak(
        PeakLegSide leg,
        double now)
    {
        if (peakEventReceiver == null)
        {
            return;
        }

        double nextAllowedTime = leg == PeakLegSide.Left
            ? nextLeftAllowedTime
            : nextRightAllowedTime;

        if (now < nextAllowedTime)
        {
            return;
        }

        double cooldown = Mathf.Max(0f, cooldownSeconds);

        if (leg == PeakLegSide.Left)
        {
            nextLeftAllowedTime = now + cooldown;
            leftPeakSubmitCount++;
            lastLeftPeakUnityTime = now;
        }
        else
        {
            nextRightAllowedTime = now + cooldown;
            rightPeakSubmitCount++;
            lastRightPeakUnityTime = now;
        }

        peakEventReceiver.SubmitPeak(leg);
    }
}
