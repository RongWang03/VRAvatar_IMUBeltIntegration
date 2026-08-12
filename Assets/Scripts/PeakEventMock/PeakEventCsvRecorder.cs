using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;

/// <summary>
/// Writes one CSV row for every applied sparse peak event.
///
/// The event is queued during LegPeakEventReceiver.PeakApplied and written
/// in LateUpdate, after the trajectory controller has processed the event.
/// This captures the event, HMD position, predicted gait values, foot targets,
/// avatar pose, anchor blending, and fallback diagnostics.
/// </summary>
public sealed class PeakEventCsvRecorder : MonoBehaviour
{
    [Header("Sources")]
    [SerializeField] private LegPeakEventReceiver receiver;

    [SerializeField]
    private PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth
        trajectoryController;

    [Tooltip("Assign CenterEyeAnchor.")]
    [SerializeField] private Transform hmdTransform;

    [Tooltip("Assign Avatar_LegIK_Test.")]
    [SerializeField] private Transform avatarRoot;

    [SerializeField] private Transform leftFootTarget;
    [SerializeField] private Transform rightFootTarget;

    [Header("Recording")]
    [SerializeField] private bool recordOnEnable = true;
    [SerializeField] private bool flushEveryRow = true;

    [Tooltip("In the Unity Editor, logs are stored beside Assets. In a build, persistentDataPath is used.")]
    [SerializeField] private string folderName = "PeakEventLogs";

    [SerializeField] private string filePrefix = "PeakEventUDP";

    [Header("Diagnostics")]
    [SerializeField] private bool isRecording;
    [SerializeField] private int rowsWritten;
    [SerializeField] private string currentFilePath = "";
    [SerializeField] private string lastError = "";

    private readonly Queue<LegPeakEventReceiver.AppliedPeakEvent>
        pendingEvents =
            new Queue<LegPeakEventReceiver.AppliedPeakEvent>();

    private StreamWriter writer;
    private double recordingStartTime;

    private FieldInfo leftEstimatedIntervalField;
    private FieldInfo rightEstimatedIntervalField;
    private FieldInfo leftEstimatedDistanceField;
    private FieldInfo rightEstimatedDistanceField;
    private FieldInfo leftPhaseField;
    private FieldInfo rightPhaseField;
    private FieldInfo leftAnchorCorrectionField;
    private FieldInfo rightAnchorCorrectionField;
    private FieldInfo leftAnchorBlendActiveField;
    private FieldInfo rightAnchorBlendActiveField;
    private FieldInfo leftFallbackLandingActiveField;
    private FieldInfo rightFallbackLandingActiveField;
    private FieldInfo leftFallbackLandingCountField;
    private FieldInfo rightFallbackLandingCountField;

    public bool IsRecording => isRecording;
    public int RowsWritten => rowsWritten;
    public string CurrentFilePath => currentFilePath;

    private void Awake()
    {
        if (receiver == null)
        {
            receiver = FindFirstObjectByType<LegPeakEventReceiver>();
        }

        if (trajectoryController == null)
        {
            trajectoryController =
                FindFirstObjectByType<
                    PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth>();
        }

        CacheTrajectoryFields();
    }

    private void OnEnable()
    {
        if (receiver != null)
        {
            receiver.PeakApplied += HandlePeakApplied;
        }

        if (recordOnEnable)
        {
            StartRecording();
        }
    }

    private void LateUpdate()
    {
        while (pendingEvents.Count > 0)
        {
            WriteEventRow(pendingEvents.Dequeue());
        }
    }

    private void HandlePeakApplied(
        LegPeakEventReceiver.AppliedPeakEvent peakEvent)
    {
        pendingEvents.Enqueue(peakEvent);
    }

    [ContextMenu("Start Recording")]
    public void StartRecording()
    {
        if (isRecording)
        {
            return;
        }

        if (receiver == null)
        {
            lastError = "LegPeakEventReceiver is not assigned.";
            Debug.LogError($"[PeakEventCsvRecorder] {lastError}", this);
            return;
        }

        try
        {
            string outputFolder = GetOutputFolder();
            Directory.CreateDirectory(outputFolder);

            string timestamp =
                DateTime.Now.ToString(
                    "yyyyMMdd_HHmmss",
                    CultureInfo.InvariantCulture);

            currentFilePath = Path.Combine(
                outputFolder,
                $"{filePrefix}_{timestamp}.csv");

            writer = new StreamWriter(
                currentFilePath,
                append: false,
                new UTF8Encoding(encoderShouldEmitUTF8Identifier: true));

            WriteHeader();

            recordingStartTime =
                Time.realtimeSinceStartupAsDouble;

            rowsWritten = 0;
            lastError = "";
            isRecording = true;

            Debug.Log(
                $"[PeakEventCsvRecorder] Recording started: {currentFilePath}",
                this);
        }
        catch (Exception exception)
        {
            lastError = exception.Message;
            isRecording = false;
            writer?.Dispose();
            writer = null;
            Debug.LogException(exception, this);
        }
    }

    [ContextMenu("Stop Recording")]
    public void StopRecording()
    {
        if (!isRecording && writer == null)
        {
            return;
        }

        try
        {
            while (pendingEvents.Count > 0)
            {
                WriteEventRow(pendingEvents.Dequeue());
            }

            writer?.Flush();
            writer?.Dispose();
        }
        catch (Exception exception)
        {
            lastError = exception.Message;
            Debug.LogException(exception, this);
        }
        finally
        {
            writer = null;
            isRecording = false;

            Debug.Log(
                $"[PeakEventCsvRecorder] Recording stopped. " +
                $"Rows={rowsWritten}, file={currentFilePath}",
                this);
        }
    }

    [ContextMenu("Open Log Folder")]
    public void OpenLogFolder()
    {
        string folder = GetOutputFolder();
        Directory.CreateDirectory(folder);
        Application.OpenURL("file://" + folder);
    }

    private void WriteHeader()
    {
        writer.WriteLine(
            "row," +
            "session_time_s," +
            "sequence," +
            "leg," +
            "source_time_s," +
            "unity_event_time_s," +
            "received_count," +
            "applied_count," +
            "reference_x," +
            "reference_y," +
            "reference_z," +
            "hmd_x," +
            "hmd_y," +
            "hmd_z," +
            "avatar_x," +
            "avatar_y," +
            "avatar_z," +
            "avatar_yaw_deg," +
            "left_target_x," +
            "left_target_y," +
            "left_target_z," +
            "right_target_x," +
            "right_target_y," +
            "right_target_z," +
            "left_interval_s," +
            "right_interval_s," +
            "left_distance_m," +
            "right_distance_m," +
            "left_phase," +
            "right_phase," +
            "left_anchor_correction_m," +
            "right_anchor_correction_m," +
            "left_anchor_blend_active," +
            "right_anchor_blend_active," +
            "left_fallback_active," +
            "right_fallback_active," +
            "left_fallback_count," +
            "right_fallback_count");
    }

    private void WriteEventRow(
        LegPeakEventReceiver.AppliedPeakEvent peakEvent)
    {
        if (!isRecording || writer == null)
        {
            return;
        }

        try
        {
            double sessionTime =
                Time.realtimeSinceStartupAsDouble -
                recordingStartTime;

            Vector3 hmd =
                hmdTransform != null
                    ? hmdTransform.position
                    : Vector3.zero;

            Vector3 avatar =
                avatarRoot != null
                    ? avatarRoot.position
                    : Vector3.zero;

            float avatarYaw =
                avatarRoot != null
                    ? avatarRoot.eulerAngles.y
                    : 0f;

            Vector3 leftTarget =
                leftFootTarget != null
                    ? leftFootTarget.position
                    : Vector3.zero;

            Vector3 rightTarget =
                rightFootTarget != null
                    ? rightFootTarget.position
                    : Vector3.zero;

            StringBuilder row = new StringBuilder(768);

            Append(row, rowsWritten + 1);
            Append(row, sessionTime);
            Append(row, peakEvent.Sequence);
            Append(row, peakEvent.Leg.ToString());
            Append(row, peakEvent.SourceTimestamp);
            Append(row, peakEvent.UnityTimestamp);
            Append(row, peakEvent.ReceivedCount);
            Append(row, peakEvent.AppliedCount);

            AppendVector(row, peakEvent.ReferencePosition);
            AppendVector(row, hmd);
            AppendVector(row, avatar);
            Append(row, avatarYaw);
            AppendVector(row, leftTarget);
            AppendVector(row, rightTarget);

            Append(row, ReadFloat(leftEstimatedIntervalField));
            Append(row, ReadFloat(rightEstimatedIntervalField));
            Append(row, ReadFloat(leftEstimatedDistanceField));
            Append(row, ReadFloat(rightEstimatedDistanceField));
            Append(row, ReadFloat(leftPhaseField));
            Append(row, ReadFloat(rightPhaseField));
            Append(row, ReadFloat(leftAnchorCorrectionField));
            Append(row, ReadFloat(rightAnchorCorrectionField));
            Append(row, ReadBool(leftAnchorBlendActiveField));
            Append(row, ReadBool(rightAnchorBlendActiveField));
            Append(row, ReadBool(leftFallbackLandingActiveField));
            Append(row, ReadBool(rightFallbackLandingActiveField));
            Append(row, ReadInt(leftFallbackLandingCountField));
            AppendLast(row, ReadInt(rightFallbackLandingCountField));

            writer.WriteLine(row.ToString());

            if (flushEveryRow)
            {
                writer.Flush();
            }

            rowsWritten++;
        }
        catch (Exception exception)
        {
            lastError = exception.Message;
            Debug.LogException(exception, this);
            StopRecording();
        }
    }

    private void CacheTrajectoryFields()
    {
        Type type = typeof(
            PeakDrivenFootTrajectoryControllerVRAnchoredSafeLandingSmooth);

        BindingFlags flags =
            BindingFlags.Instance |
            BindingFlags.NonPublic;

        leftEstimatedIntervalField =
            type.GetField("leftEstimatedInterval", flags);

        rightEstimatedIntervalField =
            type.GetField("rightEstimatedInterval", flags);

        leftEstimatedDistanceField =
            type.GetField("leftEstimatedDistance", flags);

        rightEstimatedDistanceField =
            type.GetField("rightEstimatedDistance", flags);

        leftPhaseField =
            type.GetField("leftPhase", flags);

        rightPhaseField =
            type.GetField("rightPhase", flags);

        leftAnchorCorrectionField =
            type.GetField("leftAnchorCorrection", flags);

        rightAnchorCorrectionField =
            type.GetField("rightAnchorCorrection", flags);

        leftAnchorBlendActiveField =
            type.GetField("leftAnchorBlendActive", flags);

        rightAnchorBlendActiveField =
            type.GetField("rightAnchorBlendActive", flags);

        leftFallbackLandingActiveField =
            type.GetField("leftFallbackLandingActive", flags);

        rightFallbackLandingActiveField =
            type.GetField("rightFallbackLandingActive", flags);

        leftFallbackLandingCountField =
            type.GetField("leftFallbackLandingCount", flags);

        rightFallbackLandingCountField =
            type.GetField("rightFallbackLandingCount", flags);
    }

    private float ReadFloat(FieldInfo field)
    {
        if (trajectoryController == null || field == null)
        {
            return float.NaN;
        }

        object value = field.GetValue(trajectoryController);
        return value is float result
            ? result
            : float.NaN;
    }

    private int ReadInt(FieldInfo field)
    {
        if (trajectoryController == null || field == null)
        {
            return -1;
        }

        object value = field.GetValue(trajectoryController);
        return value is int result
            ? result
            : -1;
    }

    private bool ReadBool(FieldInfo field)
    {
        if (trajectoryController == null || field == null)
        {
            return false;
        }

        object value = field.GetValue(trajectoryController);
        return value is bool result && result;
    }

    private string GetOutputFolder()
    {
#if UNITY_EDITOR
        string projectRoot =
            Path.GetFullPath(
                Path.Combine(Application.dataPath, ".."));

        return Path.Combine(projectRoot, folderName);
#else
        return Path.Combine(
            Application.persistentDataPath,
            folderName);
#endif
    }

    private static void Append(
        StringBuilder builder,
        string value)
    {
        builder.Append(Escape(value));
        builder.Append(',');
    }

    private static void Append(
        StringBuilder builder,
        int value)
    {
        builder.Append(
            value.ToString(CultureInfo.InvariantCulture));

        builder.Append(',');
    }

    private static void Append(
        StringBuilder builder,
        float value)
    {
        builder.Append(
            value.ToString(
                "F6",
                CultureInfo.InvariantCulture));

        builder.Append(',');
    }

    private static void Append(
        StringBuilder builder,
        double value)
    {
        builder.Append(
            value.ToString(
                "F6",
                CultureInfo.InvariantCulture));

        builder.Append(',');
    }

    private static void Append(
        StringBuilder builder,
        bool value)
    {
        builder.Append(value ? "1" : "0");
        builder.Append(',');
    }

    private static void AppendVector(
        StringBuilder builder,
        Vector3 value)
    {
        Append(builder, value.x);
        Append(builder, value.y);
        Append(builder, value.z);
    }

    private static void AppendLast(
        StringBuilder builder,
        int value)
    {
        builder.Append(
            value.ToString(CultureInfo.InvariantCulture));
    }

    private static string Escape(string value)
    {
        if (string.IsNullOrEmpty(value))
        {
            return "";
        }

        if (!value.Contains(",") &&
            !value.Contains("\"") &&
            !value.Contains("\n") &&
            !value.Contains("\r"))
        {
            return value;
        }

        return "\"" + value.Replace("\"", "\"\"") + "\"";
    }

    private void OnDisable()
    {
        if (receiver != null)
        {
            receiver.PeakApplied -= HandlePeakApplied;
        }

        StopRecording();
    }

    private void OnApplicationQuit()
    {
        StopRecording();
    }
}
