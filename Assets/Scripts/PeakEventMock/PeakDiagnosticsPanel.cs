using TMPro;
using UnityEngine;

/// <summary>
/// Read-only runtime diagnostics panel for the Proposed Peak Sync demo.
/// Assign all references explicitly in the Inspector.
/// </summary>
public sealed class PeakDiagnosticsPanel : MonoBehaviour
{
    [Header("Data Sources")]
    [SerializeField]
    private PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth controller;

    [SerializeField]
    private LegPeakEventReceiver receiver;

    [Header("Text Fields")]
    [SerializeField] private TMP_Text titleText;
    [SerializeField] private TMP_Text eventText;
    [SerializeField] private TMP_Text timingText;
    [SerializeField] private TMP_Text stepLengthText;
    [SerializeField] private TMP_Text blendText;

    [Header("Display")]
    [Tooltip("Refresh every frame so short prediction blends remain visible.")]
    [SerializeField] private bool refreshEveryFrame = true;

    [Min(0.02f)]
    [SerializeField] private float refreshInterval = 0.10f;

    private float nextRefreshTime;

    private void Awake()
    {
        if (titleText != null)
        {
            titleText.text = "PEAK DIAGNOSTICS";
        }
    }

    private void Update()
    {
        if (!refreshEveryFrame && Time.unscaledTime < nextRefreshTime)
        {
            return;
        }

        nextRefreshTime = Time.unscaledTime + refreshInterval;
        RefreshDisplay();
    }

    [ContextMenu("Refresh Diagnostics")]
    public void RefreshDisplay()
    {
        if (controller == null || receiver == null)
        {
            SetMissingReferenceMessage();
            return;
        }

        bool hasAppliedEvent = receiver.AppliedEventCount > 0;

        if (eventText != null)
        {
            if (hasAppliedEvent)
            {
                eventText.text =
                    $"Last Peak: {receiver.LastLeg}\n" +
                    $"Unity Time: {receiver.LastUnityTimestamp:0.000} s";
            }
            else
            {
                eventText.text =
                    "Last Peak: None\n" +
                    "Unity Time: --";
            }
        }

        if (timingText != null)
        {
            timingText.text =
                $"Interval   L {controller.LeftEstimatedInterval:0.000} s   " +
                $"R {controller.RightEstimatedInterval:0.000} s";
        }

        if (stepLengthText != null)
        {
            stepLengthText.text =
                $"Raw Step   L {controller.LeftRawEstimatedDistance:0.000} m   " +
                $"R {controller.RightRawEstimatedDistance:0.000} m\n" +
                $"Filtered   L {controller.LeftEstimatedDistance:0.000} m   " +
                $"R {controller.RightEstimatedDistance:0.000} m";
        }

        if (blendText != null)
        {
            string leftState =
                controller.LeftPredictionBlendActive ? "ON" : "OFF";

            string rightState =
                controller.RightPredictionBlendActive ? "ON" : "OFF";

            float leftProgress =
                Mathf.Clamp01(controller.LeftPredictionBlendProgress) * 100f;

            float rightProgress =
                Mathf.Clamp01(controller.RightPredictionBlendProgress) * 100f;

            blendText.text =
                $"Blend L: {leftState}  {leftProgress:0}%\n" +
                $"Blend R: {rightState}  {rightProgress:0}%";
        }
    }

    private void SetMissingReferenceMessage()
    {
        if (eventText != null)
        {
            eventText.text =
                "Missing Controller or Receiver reference.";
        }

        if (timingText != null)
        {
            timingText.text = string.Empty;
        }

        if (stepLengthText != null)
        {
            stepLengthText.text = string.Empty;
        }

        if (blendText != null)
        {
            blendText.text = string.Empty;
        }
    }
}