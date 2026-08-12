using System;
using System.Globalization;
using System.IO;
using UnityEngine;

/// <summary>
/// Writes one CSV row for each predicted foot landing raised by
/// PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth.
/// </summary>
[DefaultExecutionOrder(10002)]
public sealed class PeakLandingCsvRecorder : MonoBehaviour
{
    [Header("Source")]
    [SerializeField]
    private PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth
        trajectoryController;

    [Header("Recording")]
    [SerializeField] private bool recordOnStart = true;

    [Tooltip("Folder created inside the Unity project root.")]
    [SerializeField] private string outputFolderName = "RecordedData";

    [SerializeField] private bool logFilePath = true;

    [Header("Diagnostics")]
    [SerializeField] private bool isRecording;
    [SerializeField] private int rowsWritten;
    [SerializeField] private string currentFilePath;

    private StreamWriter writer;
    private int landingSequence;

    private void Awake()
    {
        if (trajectoryController == null)
        {
            trajectoryController =
                FindFirstObjectByType<
                    PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth>();
        }
    }

    private void OnEnable()
    {
        Subscribe();

        if (recordOnStart)
        {
            StartRecording();
        }
    }

    private void OnDisable()
    {
        Unsubscribe();
        StopRecording();
    }

    private void OnApplicationQuit()
    {
        StopRecording();
    }

    [ContextMenu("Start Recording")]
    public void StartRecording()
    {
        if (isRecording)
        {
            return;
        }

        if (trajectoryController == null)
        {
            Debug.LogError(
                "[PeakLandingCsvRecorder] Trajectory Controller is not assigned.",
                this);

            return;
        }

        string projectRoot =
            Directory.GetParent(Application.dataPath)?.FullName
            ?? Application.dataPath;

        string folderPath =
            Path.Combine(projectRoot, outputFolderName);

        Directory.CreateDirectory(folderPath);

        string fileName =
            $"PeakLandingEvents_{DateTime.Now:yyyyMMdd_HHmmss}.csv";

        currentFilePath =
            Path.Combine(folderPath, fileName);

        writer = new StreamWriter(
            currentFilePath,
            false);

        writer.WriteLine(
            "landing_sequence," +
            "leg," +
            "unity_time," +
            "world_x," +
            "world_y," +
            "world_z," +
            "landing_reason");

        writer.Flush();

        landingSequence = 0;
        rowsWritten = 0;
        isRecording = true;

        if (logFilePath)
        {
            Debug.Log(
                $"[PeakLandingCsvRecorder] Recording started: {currentFilePath}",
                this);
        }
    }

    [ContextMenu("Stop Recording")]
    public void StopRecording()
    {
        if (!isRecording && writer == null)
        {
            return;
        }

        isRecording = false;

        writer?.Flush();
        writer?.Dispose();
        writer = null;

        if (logFilePath &&
            !string.IsNullOrWhiteSpace(currentFilePath))
        {
            Debug.Log(
                $"[PeakLandingCsvRecorder] Recording stopped. " +
                $"Rows={rowsWritten}, File={currentFilePath}",
                this);
        }
    }

    private void Subscribe()
    {
        if (trajectoryController != null)
        {
            trajectoryController.FootLanded -= HandleFootLanded;
            trajectoryController.FootLanded += HandleFootLanded;
        }
    }

    private void Unsubscribe()
    {
        if (trajectoryController != null)
        {
            trajectoryController.FootLanded -= HandleFootLanded;
        }
    }

    private void HandleFootLanded(
        PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth
            .FootLandingEvent landingEvent)
    {
        if (!isRecording || writer == null)
        {
            return;
        }

        landingSequence++;

        Vector3 position =
            landingEvent.WorldPosition;

        writer.WriteLine(
            string.Join(
                ",",
                landingSequence.ToString(
                    CultureInfo.InvariantCulture),

                landingEvent.Leg.ToString(),

                landingEvent.UnityTimestamp.ToString(
                    "F6",
                    CultureInfo.InvariantCulture),

                position.x.ToString(
                    "F6",
                    CultureInfo.InvariantCulture),

                position.y.ToString(
                    "F6",
                    CultureInfo.InvariantCulture),

                position.z.ToString(
                    "F6",
                    CultureInfo.InvariantCulture),

                landingEvent.Reason.ToString()));

        writer.Flush();
        rowsWritten++;
    }
}
