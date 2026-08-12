using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Writes one CSV row for every completed left/right stance period.
///
/// This is an analysis-only recorder. It does not modify IK, SmoothAnchor,
/// SafeLanding, UDP input, or foot-trajectory behaviour.
///
/// A stance starts when the Foot Target enters the initial ground-height band.
/// A stance ends when the Foot Target leaves that band.
/// </summary>
[DefaultExecutionOrder(10001)]
public sealed class PeakStanceSummaryCsvRecorder : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private LegPeakEventReceiver receiver;

    [SerializeField] private Transform leftFootTarget;
    [SerializeField] private Transform rightFootTarget;

    [SerializeField] private Transform leftFootBone;
    [SerializeField] private Transform rightFootBone;

    [Header("Recording")]
    [SerializeField] private bool recordOnEnable = true;
    [SerializeField] private bool flushEveryRow = true;

    [SerializeField] private string folderName = "PeakEventLogs";
    [SerializeField] private string filePrefix = "PeakStanceSummary";

    [Header("Stance Detection")]
    [Min(0.001f)]
    [SerializeField] private float stanceHeightThreshold = 0.015f;

    [Tooltip("Ignore extremely short stance segments caused by a one-frame transition.")]
    [Min(0f)]
    [SerializeField] private float minimumStanceDuration = 0.05f;

    [Header("Diagnostics")]
    [SerializeField] private bool isRecording;
    [SerializeField] private int completedStanceCount;
    [SerializeField] private int leftCompletedStanceCount;
    [SerializeField] private int rightCompletedStanceCount;
    [SerializeField] private string currentFilePath = "";
    [SerializeField] private string lastError = "";

    private StreamWriter writer;
    private double recordingStartTime;

    private bool groundHeightsInitialized;
    private float leftGroundY;
    private float rightGroundY;

    private FootStanceState leftState;
    private FootStanceState rightState;

    public bool IsRecording => isRecording;
    public int CompletedStanceCount => completedStanceCount;
    public string CurrentFilePath => currentFilePath;

    private void Awake()
    {
        if (receiver == null)
        {
            receiver = FindFirstObjectByType<LegPeakEventReceiver>();
        }
    }

    private void OnEnable()
    {
        if (recordOnEnable)
        {
            StartRecording();
        }
    }

    private void LateUpdate()
    {
        if (!isRecording || writer == null)
        {
            return;
        }

        if (leftFootTarget == null ||
            rightFootTarget == null ||
            leftFootBone == null ||
            rightFootBone == null)
        {
            return;
        }

        double now = Time.realtimeSinceStartupAsDouble;

        Vector3 leftTarget = leftFootTarget.position;
        Vector3 rightTarget = rightFootTarget.position;
        Vector3 leftBone = leftFootBone.position;
        Vector3 rightBone = rightFootBone.position;

        if (!groundHeightsInitialized)
        {
            leftGroundY = leftTarget.y;
            rightGroundY = rightTarget.y;
            groundHeightsInitialized = true;
        }

        bool leftNearGround =
            Mathf.Abs(leftTarget.y - leftGroundY)
            <= stanceHeightThreshold;

        bool rightNearGround =
            Mathf.Abs(rightTarget.y - rightGroundY)
            <= stanceHeightThreshold;

        UpdateFootState(
            "Left",
            ref leftState,
            leftNearGround,
            leftTarget,
            leftBone,
            now);

        UpdateFootState(
            "Right",
            ref rightState,
            rightNearGround,
            rightTarget,
            rightBone,
            now);
    }

    [ContextMenu("Start Recording")]
    public void StartRecording()
    {
        if (isRecording)
        {
            return;
        }

        try
        {
            string outputFolder = GetOutputFolder();
            Directory.CreateDirectory(outputFolder);

            string timestamp = DateTime.Now.ToString(
                "yyyyMMdd_HHmmss",
                CultureInfo.InvariantCulture);

            currentFilePath = Path.Combine(
                outputFolder,
                $"{filePrefix}_{timestamp}.csv");

            writer = new StreamWriter(
                currentFilePath,
                false,
                new UTF8Encoding(true));

            WriteHeader();

            recordingStartTime =
                Time.realtimeSinceStartupAsDouble;

            completedStanceCount = 0;
            leftCompletedStanceCount = 0;
            rightCompletedStanceCount = 0;

            groundHeightsInitialized = false;
            leftGroundY = 0f;
            rightGroundY = 0f;

            leftState = new FootStanceState();
            rightState = new FootStanceState();

            lastError = "";
            isRecording = true;

            Debug.Log(
                $"[PeakStanceSummaryCsvRecorder] Recording started: " +
                currentFilePath,
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
            double now = Time.realtimeSinceStartupAsDouble;

            if (leftState.IsActive)
            {
                CompleteStance(
                    "Left",
                    ref leftState,
                    now,
                    wasOpenAtStop: true);
            }

            if (rightState.IsActive)
            {
                CompleteStance(
                    "Right",
                    ref rightState,
                    now,
                    wasOpenAtStop: true);
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
                $"[PeakStanceSummaryCsvRecorder] Recording stopped. " +
                $"Completed={completedStanceCount}, " +
                $"Left={leftCompletedStanceCount}, " +
                $"Right={rightCompletedStanceCount}, " +
                $"File={currentFilePath}",
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

    private void UpdateFootState(
        string leg,
        ref FootStanceState state,
        bool nearGround,
        Vector3 targetPosition,
        Vector3 bonePosition,
        double now)
    {
        if (nearGround && !state.IsActive)
        {
            BeginStance(
                ref state,
                targetPosition,
                bonePosition,
                now);
        }

        if (nearGround && state.IsActive)
        {
            UpdateActiveStance(
                ref state,
                targetPosition,
                bonePosition);
        }

        if (!nearGround && state.IsActive)
        {
            CompleteStance(
                leg,
                ref state,
                now,
                wasOpenAtStop: false);
        }
    }

    private static void BeginStance(
        ref FootStanceState state,
        Vector3 targetPosition,
        Vector3 bonePosition,
        double now)
    {
        state.IsActive = true;
        state.StartTime = now;
        state.FrameCount = 0;

        state.TargetStart = targetPosition;
        state.TargetPrevious = targetPosition;
        state.TargetEnd = targetPosition;
        state.TargetPathLength = 0f;
        state.TargetMaxDisplacement = 0f;

        state.BoneStart = bonePosition;
        state.BonePrevious = bonePosition;
        state.BoneEnd = bonePosition;
        state.BonePathLength = 0f;
        state.BoneMaxDisplacement = 0f;

        float initialError =
            Vector3.Distance(targetPosition, bonePosition);

        state.ErrorSum = initialError;
        state.ErrorMaximum = initialError;
        state.ErrorSampleCount = 1;
    }

    private static void UpdateActiveStance(
        ref FootStanceState state,
        Vector3 targetPosition,
        Vector3 bonePosition)
    {
        state.FrameCount++;

        state.TargetPathLength +=
            Vector3.Distance(
                targetPosition,
                state.TargetPrevious);

        state.BonePathLength +=
            Vector3.Distance(
                bonePosition,
                state.BonePrevious);

        state.TargetEnd = targetPosition;
        state.BoneEnd = bonePosition;

        state.TargetMaxDisplacement =
            Mathf.Max(
                state.TargetMaxDisplacement,
                Vector3.Distance(
                    targetPosition,
                    state.TargetStart));

        state.BoneMaxDisplacement =
            Mathf.Max(
                state.BoneMaxDisplacement,
                Vector3.Distance(
                    bonePosition,
                    state.BoneStart));

        float error =
            Vector3.Distance(targetPosition, bonePosition);

        state.ErrorSum += error;
        state.ErrorMaximum =
            Mathf.Max(state.ErrorMaximum, error);

        state.ErrorSampleCount++;

        state.TargetPrevious = targetPosition;
        state.BonePrevious = bonePosition;
    }

    private void CompleteStance(
        string leg,
        ref FootStanceState state,
        double now,
        bool wasOpenAtStop)
    {
        double duration = now - state.StartTime;

        if (duration >= minimumStanceDuration)
        {
            WriteStanceRow(
                leg,
                state,
                now,
                duration,
                wasOpenAtStop);

            completedStanceCount++;

            if (leg == "Left")
            {
                leftCompletedStanceCount++;
            }
            else
            {
                rightCompletedStanceCount++;
            }
        }

        state = new FootStanceState();
    }

    private void WriteHeader()
    {
        writer.WriteLine(
            "stance_index," +
            "leg," +
            "start_time_s," +
            "end_time_s," +
            "duration_s," +
            "frame_count," +
            "closed_by_recording_stop," +
            "received_event_count," +
            "applied_event_count," +
            "last_sequence," +
            "last_leg," +
            "target_start_x," +
            "target_start_y," +
            "target_start_z," +
            "target_end_x," +
            "target_end_y," +
            "target_end_z," +
            "target_end_displacement_m," +
            "target_max_displacement_m," +
            "target_path_length_m," +
            "bone_start_x," +
            "bone_start_y," +
            "bone_start_z," +
            "bone_end_x," +
            "bone_end_y," +
            "bone_end_z," +
            "bone_end_displacement_m," +
            "bone_max_displacement_m," +
            "bone_path_length_m," +
            "mean_target_error_m," +
            "max_target_error_m");
    }

    private void WriteStanceRow(
        string leg,
        FootStanceState state,
        double now,
        double duration,
        bool wasOpenAtStop)
    {
        double startTime =
            state.StartTime - recordingStartTime;

        double endTime =
            now - recordingStartTime;

        float targetEndDisplacement =
            Vector3.Distance(
                state.TargetEnd,
                state.TargetStart);

        float boneEndDisplacement =
            Vector3.Distance(
                state.BoneEnd,
                state.BoneStart);

        float meanError =
            state.ErrorSampleCount > 0
                ? state.ErrorSum / state.ErrorSampleCount
                : 0f;

        StringBuilder row = new StringBuilder(768);

        Append(row, completedStanceCount + 1);
        Append(row, leg);
        Append(row, startTime);
        Append(row, endTime);
        Append(row, duration);
        Append(row, state.FrameCount);
        Append(row, wasOpenAtStop);

        Append(row, GetReceiverInt("ReceivedEventCount"));
        Append(row, GetReceiverInt("AppliedEventCount"));
        Append(row, GetReceiverInt("LastSequence"));
        Append(row, GetReceiverString("LastLeg"));

        AppendVector(row, state.TargetStart);
        AppendVector(row, state.TargetEnd);

        Append(row, targetEndDisplacement);
        Append(row, state.TargetMaxDisplacement);
        Append(row, state.TargetPathLength);

        AppendVector(row, state.BoneStart);
        AppendVector(row, state.BoneEnd);

        Append(row, boneEndDisplacement);
        Append(row, state.BoneMaxDisplacement);
        Append(row, state.BonePathLength);

        Append(row, meanError);
        AppendLast(row, state.ErrorMaximum);

        writer.WriteLine(row.ToString());

        if (flushEveryRow)
        {
            writer.Flush();
        }
    }

    private int GetReceiverInt(string propertyName)
    {
        if (receiver == null)
        {
            return -1;
        }

        var property =
            receiver.GetType().GetProperty(propertyName);

        if (property != null &&
            property.PropertyType == typeof(int))
        {
            return (int)property.GetValue(receiver);
        }

        return -1;
    }

    private string GetReceiverString(string propertyName)
    {
        if (receiver == null)
        {
            return "";
        }

        var property =
            receiver.GetType().GetProperty(propertyName);

        if (property == null)
        {
            return "";
        }

        object value = property.GetValue(receiver);
        return value?.ToString() ?? "";
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
        float value)
    {
        builder.Append(
            value.ToString(
                "F6",
                CultureInfo.InvariantCulture));
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

        return "\"" +
               value.Replace("\"", "\"\"") +
               "\"";
    }

    private void OnDisable()
    {
        StopRecording();
    }

    private void OnApplicationQuit()
    {
        StopRecording();
    }

    private struct FootStanceState
    {
        public bool IsActive;
        public double StartTime;
        public int FrameCount;

        public Vector3 TargetStart;
        public Vector3 TargetPrevious;
        public Vector3 TargetEnd;
        public float TargetPathLength;
        public float TargetMaxDisplacement;

        public Vector3 BoneStart;
        public Vector3 BonePrevious;
        public Vector3 BoneEnd;
        public float BonePathLength;
        public float BoneMaxDisplacement;

        public float ErrorSum;
        public float ErrorMaximum;
        public int ErrorSampleCount;
    }
}
