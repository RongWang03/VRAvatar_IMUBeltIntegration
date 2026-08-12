using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Records IK pose data every Unity frame after the IK update.
/// Use this together with PeakEventCsvRecorder:
/// - PeakEventCsvRecorder: one row per UDP peak event
/// - PeakPoseFrameCsvRecorder: one row per Unity pose frame
/// </summary>
[DefaultExecutionOrder(10000)]
public sealed class PeakPoseFrameCsvRecorder : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private LegPeakEventReceiver receiver;

    [SerializeField] private Transform avatarRoot;
    [SerializeField] private Transform hmdTransform;

    [SerializeField] private Transform leftFootTarget;
    [SerializeField] private Transform rightFootTarget;

    [SerializeField] private Transform leftFootBone;
    [SerializeField] private Transform rightFootBone;

    [Header("Recording")]
    [SerializeField] private bool recordOnEnable = true;

    [Tooltip("0 means record every rendered frame.")]
    [Min(0f)]
    [SerializeField] private float targetRecordRateHz = 0f;

    [Min(1)]
    [SerializeField] private int flushEveryRows = 60;

    [SerializeField] private string folderName = "PeakEventLogs";
    [SerializeField] private string filePrefix = "PeakPoseFrames";

    [Header("Stance Detection")]
    [Tooltip("A foot is considered near the ground below this vertical distance.")]
    [Min(0.001f)]
    [SerializeField] private float stanceHeightThreshold = 0.015f;

    [Tooltip("Phase range used as the expected world-space stance period.")]
    [Range(0f, 1f)]
    [SerializeField] private float stancePhaseStart = 0.25f;

    [Range(0f, 1f)]
    [SerializeField] private float stancePhaseEnd = 0.75f;

    [Header("Diagnostics")]
    [SerializeField] private bool isRecording;
    [SerializeField] private int rowsWritten;
    [SerializeField] private float recordingElapsedSeconds;
    [SerializeField] private float measuredRecordRateHz;
    [SerializeField] private string currentFilePath = "";
    [SerializeField] private string lastError = "";

    private StreamWriter writer;
    private double recordingStartTime;
    private double nextRecordTime;

    private Vector3 previousLeftTargetPosition;
    private Vector3 previousRightTargetPosition;
    private Vector3 previousLeftBonePosition;
    private Vector3 previousRightBonePosition;

    private bool hasPreviousFrame;

    // Ground height is captured once at the start of recording.
    // It must not follow the moving Foot Target or Foot Bone.
    private bool groundHeightsInitialized;
    private float leftGroundY;
    private float rightGroundY;

    private bool previousLeftNearGround;
    private bool previousRightNearGround;

    private Vector3 leftTargetStanceStartPosition;
    private Vector3 rightTargetStanceStartPosition;
    private Vector3 leftBoneStanceStartPosition;
    private Vector3 rightBoneStanceStartPosition;

    private float leftTargetCurrentStanceDisplacement;
    private float rightTargetCurrentStanceDisplacement;
    private float leftBoneCurrentStanceDisplacement;
    private float rightBoneCurrentStanceDisplacement;

    private float leftTargetMaxStanceDisplacement;
    private float rightTargetMaxStanceDisplacement;
    private float leftBoneMaxStanceDisplacement;
    private float rightBoneMaxStanceDisplacement;

    private float leftTargetStanceDrift;
    private float rightTargetStanceDrift;
    private float leftBoneStanceDrift;
    private float rightBoneStanceDrift;

    public bool IsRecording => isRecording;
    public int RowsWritten => rowsWritten;
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

        double now = Time.realtimeSinceStartupAsDouble;

        if (targetRecordRateHz > 0f)
        {
            double interval = 1.0 / targetRecordRateHz;

            if (now < nextRecordTime)
            {
                return;
            }

            nextRecordTime += interval;

            if (nextRecordTime < now - interval)
            {
                nextRecordTime = now + interval;
            }
        }

        WritePoseRow(now);
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

            recordingStartTime = Time.realtimeSinceStartupAsDouble;
            nextRecordTime = recordingStartTime;

            rowsWritten = 0;
            recordingElapsedSeconds = 0f;
            measuredRecordRateHz = 0f;
            lastError = "";

            hasPreviousFrame = false;

            groundHeightsInitialized = false;
            leftGroundY = 0f;
            rightGroundY = 0f;

            previousLeftNearGround = false;
            previousRightNearGround = false;

            leftTargetStanceStartPosition = Vector3.zero;
            rightTargetStanceStartPosition = Vector3.zero;
            leftBoneStanceStartPosition = Vector3.zero;
            rightBoneStanceStartPosition = Vector3.zero;

            leftTargetCurrentStanceDisplacement = 0f;
            rightTargetCurrentStanceDisplacement = 0f;
            leftBoneCurrentStanceDisplacement = 0f;
            rightBoneCurrentStanceDisplacement = 0f;

            leftTargetMaxStanceDisplacement = 0f;
            rightTargetMaxStanceDisplacement = 0f;
            leftBoneMaxStanceDisplacement = 0f;
            rightBoneMaxStanceDisplacement = 0f;

            leftTargetStanceDrift = 0f;
            rightTargetStanceDrift = 0f;
            leftBoneStanceDrift = 0f;
            rightBoneStanceDrift = 0f;

            isRecording = true;

            Debug.Log(
                $"[PeakPoseFrameCsvRecorder] Recording started: {currentFilePath}",
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
                $"[PeakPoseFrameCsvRecorder] Recording stopped. " +
                $"Rows={rowsWritten}, Rate={measuredRecordRateHz:F2} Hz, " +
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

    private void WriteHeader()
    {
        writer.WriteLine(
            "row," +
            "recording_time_s," +
            "unity_time_s," +
            "unity_frame," +
            "received_event_count," +
            "applied_event_count," +
            "last_sequence," +
            "last_leg," +
            "avatar_x," +
            "avatar_y," +
            "avatar_z," +
            "avatar_yaw_deg," +
            "hmd_x," +
            "hmd_y," +
            "hmd_z," +
            "left_target_x," +
            "left_target_y," +
            "left_target_z," +
            "right_target_x," +
            "right_target_y," +
            "right_target_z," +
            "left_bone_x," +
            "left_bone_y," +
            "left_bone_z," +
            "right_bone_x," +
            "right_bone_y," +
            "right_bone_z," +
            "left_target_error_m," +
            "right_target_error_m," +
            "left_target_near_ground," +
            "right_target_near_ground," +
            "left_target_frame_motion_m," +
            "right_target_frame_motion_m," +
            "left_bone_frame_motion_m," +
            "right_bone_frame_motion_m," +
            "left_target_stance_start_x," +
            "left_target_stance_start_y," +
            "left_target_stance_start_z," +
            "right_target_stance_start_x," +
            "right_target_stance_start_y," +
            "right_target_stance_start_z," +
            "left_bone_stance_start_x," +
            "left_bone_stance_start_y," +
            "left_bone_stance_start_z," +
            "right_bone_stance_start_x," +
            "right_bone_stance_start_y," +
            "right_bone_stance_start_z," +
            "left_target_current_stance_displacement_m," +
            "right_target_current_stance_displacement_m," +
            "left_bone_current_stance_displacement_m," +
            "right_bone_current_stance_displacement_m," +
            "left_target_max_stance_displacement_m," +
            "right_target_max_stance_displacement_m," +
            "left_bone_max_stance_displacement_m," +
            "right_bone_max_stance_displacement_m," +
            "left_target_stance_drift_m," +
            "right_target_stance_drift_m," +
            "left_bone_stance_drift_m," +
            "right_bone_stance_drift_m");
    }

    private void WritePoseRow(double now)
    {
        Vector3 avatar = GetPosition(avatarRoot);
        Vector3 hmd = GetPosition(hmdTransform);

        Vector3 leftTarget = GetPosition(leftFootTarget);
        Vector3 rightTarget = GetPosition(rightFootTarget);

        Vector3 leftBone = GetPosition(leftFootBone);
        Vector3 rightBone = GetPosition(rightFootBone);

        float leftTargetError =
            Vector3.Distance(leftTarget, leftBone);

        float rightTargetError =
            Vector3.Distance(rightTarget, rightBone);

        if (!groundHeightsInitialized)
        {
            leftGroundY = leftTarget.y;
            rightGroundY = rightTarget.y;
            groundHeightsInitialized = true;
        }

        bool leftNearGround =
            IsNearGround(leftTarget.y, leftGroundY);

        bool rightNearGround =
            IsNearGround(rightTarget.y, rightGroundY);

        if (leftNearGround && !previousLeftNearGround)
        {
            leftTargetStanceStartPosition = leftTarget;
            leftBoneStanceStartPosition = leftBone;

            leftTargetCurrentStanceDisplacement = 0f;
            leftBoneCurrentStanceDisplacement = 0f;

            leftTargetMaxStanceDisplacement = 0f;
            leftBoneMaxStanceDisplacement = 0f;

            leftTargetStanceDrift = 0f;
            leftBoneStanceDrift = 0f;
        }
        else if (!leftNearGround)
        {
            leftTargetCurrentStanceDisplacement = 0f;
            leftBoneCurrentStanceDisplacement = 0f;
        }

        if (rightNearGround && !previousRightNearGround)
        {
            rightTargetStanceStartPosition = rightTarget;
            rightBoneStanceStartPosition = rightBone;

            rightTargetCurrentStanceDisplacement = 0f;
            rightBoneCurrentStanceDisplacement = 0f;

            rightTargetMaxStanceDisplacement = 0f;
            rightBoneMaxStanceDisplacement = 0f;

            rightTargetStanceDrift = 0f;
            rightBoneStanceDrift = 0f;
        }
        else if (!rightNearGround)
        {
            rightTargetCurrentStanceDisplacement = 0f;
            rightBoneCurrentStanceDisplacement = 0f;
        }

        if (leftNearGround)
        {
            leftTargetCurrentStanceDisplacement =
                Vector3.Distance(
                    leftTarget,
                    leftTargetStanceStartPosition);

            leftBoneCurrentStanceDisplacement =
                Vector3.Distance(
                    leftBone,
                    leftBoneStanceStartPosition);

            leftTargetMaxStanceDisplacement =
                Mathf.Max(
                    leftTargetMaxStanceDisplacement,
                    leftTargetCurrentStanceDisplacement);

            leftBoneMaxStanceDisplacement =
                Mathf.Max(
                    leftBoneMaxStanceDisplacement,
                    leftBoneCurrentStanceDisplacement);
        }

        if (rightNearGround)
        {
            rightTargetCurrentStanceDisplacement =
                Vector3.Distance(
                    rightTarget,
                    rightTargetStanceStartPosition);

            rightBoneCurrentStanceDisplacement =
                Vector3.Distance(
                    rightBone,
                    rightBoneStanceStartPosition);

            rightTargetMaxStanceDisplacement =
                Mathf.Max(
                    rightTargetMaxStanceDisplacement,
                    rightTargetCurrentStanceDisplacement);

            rightBoneMaxStanceDisplacement =
                Mathf.Max(
                    rightBoneMaxStanceDisplacement,
                    rightBoneCurrentStanceDisplacement);
        }

        float leftTargetFrameMotion = 0f;
        float rightTargetFrameMotion = 0f;
        float leftBoneFrameMotion = 0f;
        float rightBoneFrameMotion = 0f;

        if (hasPreviousFrame)
        {
            leftTargetFrameMotion =
                Vector3.Distance(leftTarget, previousLeftTargetPosition);

            rightTargetFrameMotion =
                Vector3.Distance(rightTarget, previousRightTargetPosition);

            leftBoneFrameMotion =
                Vector3.Distance(leftBone, previousLeftBonePosition);

            rightBoneFrameMotion =
                Vector3.Distance(rightBone, previousRightBonePosition);

            if (leftNearGround)
            {
                leftTargetStanceDrift += leftTargetFrameMotion;
                leftBoneStanceDrift += leftBoneFrameMotion;
            }
            else
            {
                leftTargetStanceDrift = 0f;
                leftBoneStanceDrift = 0f;
            }

            if (rightNearGround)
            {
                rightTargetStanceDrift += rightTargetFrameMotion;
                rightBoneStanceDrift += rightBoneFrameMotion;
            }
            else
            {
                rightTargetStanceDrift = 0f;
                rightBoneStanceDrift = 0f;
            }
        }

        double recordingTime = now - recordingStartTime;

        StringBuilder row = new StringBuilder(1024);

        Append(row, rowsWritten + 1);
        Append(row, recordingTime);
        Append(row, now);
        Append(row, Time.frameCount);

        Append(row, GetReceiverInt("ReceivedEventCount"));
        Append(row, GetReceiverInt("AppliedEventCount"));
        Append(row, GetReceiverInt("LastSequence"));
        Append(row, GetReceiverString("LastLeg"));

        AppendVector(row, avatar);
        Append(row, avatarRoot != null ? avatarRoot.eulerAngles.y : 0f);
        AppendVector(row, hmd);

        AppendVector(row, leftTarget);
        AppendVector(row, rightTarget);

        AppendVector(row, leftBone);
        AppendVector(row, rightBone);

        Append(row, leftTargetError);
        Append(row, rightTargetError);

        Append(row, leftNearGround);
        Append(row, rightNearGround);

        Append(row, leftTargetFrameMotion);
        Append(row, rightTargetFrameMotion);
        Append(row, leftBoneFrameMotion);
        Append(row, rightBoneFrameMotion);

        AppendVector(row, leftTargetStanceStartPosition);
        AppendVector(row, rightTargetStanceStartPosition);
        AppendVector(row, leftBoneStanceStartPosition);
        AppendVector(row, rightBoneStanceStartPosition);

        Append(row, leftTargetCurrentStanceDisplacement);
        Append(row, rightTargetCurrentStanceDisplacement);
        Append(row, leftBoneCurrentStanceDisplacement);
        Append(row, rightBoneCurrentStanceDisplacement);

        Append(row, leftTargetMaxStanceDisplacement);
        Append(row, rightTargetMaxStanceDisplacement);
        Append(row, leftBoneMaxStanceDisplacement);
        Append(row, rightBoneMaxStanceDisplacement);

        Append(row, leftTargetStanceDrift);
        Append(row, rightTargetStanceDrift);
        Append(row, leftBoneStanceDrift);
        AppendLast(row, rightBoneStanceDrift);

        writer.WriteLine(row.ToString());

        rowsWritten++;
        recordingElapsedSeconds = (float)recordingTime;

        measuredRecordRateHz =
            recordingElapsedSeconds > 0f
                ? rowsWritten / recordingElapsedSeconds
                : 0f;

        if (rowsWritten % Mathf.Max(1, flushEveryRows) == 0)
        {
            writer.Flush();
        }

        previousLeftTargetPosition = leftTarget;
        previousRightTargetPosition = rightTarget;
        previousLeftBonePosition = leftBone;
        previousRightBonePosition = rightBone;

        previousLeftNearGround = leftNearGround;
        previousRightNearGround = rightNearGround;

        hasPreviousFrame = true;
    }

    private bool IsNearGround(
        float targetY,
        float groundY)
    {
        return Mathf.Abs(targetY - groundY)
            <= stanceHeightThreshold;
    }

    private int GetReceiverInt(string propertyName)
    {
        if (receiver == null)
        {
            return -1;
        }

        var property = receiver.GetType().GetProperty(propertyName);

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

        var property = receiver.GetType().GetProperty(propertyName);

        if (property == null)
        {
            return "";
        }

        object value = property.GetValue(receiver);
        return value?.ToString() ?? "";
    }

    private static Vector3 GetPosition(Transform target)
    {
        return target != null
            ? target.position
            : Vector3.zero;
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
            value.ToString("F6", CultureInfo.InvariantCulture));

        builder.Append(',');
    }

    private static void Append(
        StringBuilder builder,
        double value)
    {
        builder.Append(
            value.ToString("F6", CultureInfo.InvariantCulture));

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
            value.ToString("F6", CultureInfo.InvariantCulture));
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
        StopRecording();
    }

    private void OnApplicationQuit()
    {
        StopRecording();
    }
}