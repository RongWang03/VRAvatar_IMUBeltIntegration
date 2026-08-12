using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Threading;
using UnityEngine;

// Peak事件只表达“哪一条腿到达了峰值”，不负责解释UDP/按键协议。
// 所有输入适配器最终都应把外部信号转换成这个统一枚举。

public enum PeakLegSide
{
    Left,
    Right
}

/// <summary>
/// Unified input layer for sparse leg-peak events.
/// A future UDP script can call SubmitPeak when it receives
/// "left is up" or "right is up".
/// </summary>
public sealed class LegPeakEventReceiver : MonoBehaviour
{
    // ===== 事件发生时的位置参考 =====
    // 通常绑定CenterEyeAnchor/HMD。ApplyPeak会在Unity主线程读取其世界位置，
    // 供下游控制器估算相邻同腿Peak之间的水平位移与预测步长。

    [Header("Reference Position")]
    [Tooltip("Desktop test: Avatar root. VR test: CenterEyeAnchor / HMD camera.")]
    [SerializeField] private Transform referenceTransform;

    // ===== Inspector运行时诊断 =====
    // 这些字段只记录接收数量、应用数量和最近事件，不生成腿部轨迹。

    [Header("Diagnostics")]
    [SerializeField] private bool logEvents = true;
    [SerializeField] private int receivedEventCount;
    [SerializeField] private int appliedEventCount;
    [SerializeField] private int lastSequence = -1;
    [SerializeField] private PeakLegSide lastLeg;
    [SerializeField] private double lastSourceTimestamp;
    [SerializeField] private double lastUnityTimestamp;
    [SerializeField] private Vector3 lastReferencePosition;

    // ===== 线程安全事件队列 =====
    // UDP接收线程等后台来源只能负责入队；不能直接调用Unity Transform API。
    // Update随后在Unity主线程逐个取出并应用事件。

    private readonly ConcurrentQueue<QueuedPeakEvent> pendingEvents =
        new ConcurrentQueue<QueuedPeakEvent>();

    // ===== 统一的对外Peak事件 =====
    // Smooth Controller、CSV等组件订阅此事件；Receiver本身不移动Foot Target。

    public event Action<AppliedPeakEvent> PeakApplied;

    public int ReceivedEventCount => receivedEventCount;
    public int AppliedEventCount => appliedEventCount;
    public int LastSequence => lastSequence;
    public PeakLegSide LastLeg => lastLeg;
    public double LastSourceTimestamp => lastSourceTimestamp;
    public double LastUnityTimestamp => lastUnityTimestamp;
    public Vector3 LastReferencePosition => lastReferencePosition;
    public Transform ReferenceTransform => referenceTransform;

    // ===== 主线程事件泵 =====
    // 每帧清空当前队列；每个QueuedPeakEvent都通过ApplyPeak转换成AppliedPeakEvent。

    private void Update()
    {
        while (pendingEvents.TryDequeue(out QueuedPeakEvent queuedEvent))
        {
            ApplyPeak(queuedEvent);
        }
    }

    /// <summary>
    /// Thread-safe entry point for a tester, file player, or UDP receiver.
    /// </summary>
    // ===== 所有输入方式的统一入口 =====
    // UDP、Quest、键盘和未来Jasmine Adapter都应调用这里。
    // 本方法只补齐时间戳、入队并更新接收计数，不直接触发步态。

    public void SubmitPeak(
        PeakLegSide leg,
        double sourceTimestamp = -1.0,
        int sequence = -1)
    {
        if (sourceTimestamp < 0.0)
        {
            sourceTimestamp = GetMonotonicSeconds();
        }

        pendingEvents.Enqueue(
            new QueuedPeakEvent(leg, sourceTimestamp, sequence));

        Interlocked.Increment(ref receivedEventCount);
    }

    // UnityEvent/Inspector可直接绑定的左腿便捷入口。

    public void SubmitLeftPeak()
    {
        SubmitPeak(PeakLegSide.Left);
    }

    // UnityEvent/Inspector可直接绑定的右腿便捷入口。

    public void SubmitRightPeak()
    {
        SubmitPeak(PeakLegSide.Right);
    }

    // ===== 手动重置诊断状态 =====
    // 清空尚未应用的事件并重置计数；不会修改Smooth Controller或Foot Target。

    [ContextMenu("Reset Receiver")]
    public void ResetReceiver()
    {
        while (pendingEvents.TryDequeue(out _))
        {
        }

        receivedEventCount = 0;
        appliedEventCount = 0;
        lastSequence = -1;
        lastLeg = PeakLegSide.Left;
        lastSourceTimestamp = 0.0;
        lastUnityTimestamp = 0.0;
        lastReferencePosition = Vector3.zero;
    }

    // ===== 在Unity主线程正式应用一个Peak =====
    // 采集Unity时间和HMD参考位置，生成不可变AppliedPeakEvent，
    // 然后通过PeakApplied通知Smooth Controller及所有观察者。

    private void ApplyPeak(QueuedPeakEvent queuedEvent)
    {
        double unityTimestamp = Time.realtimeSinceStartupAsDouble;

        Vector3 referencePosition = referenceTransform != null
            ? referenceTransform.position
            : Vector3.zero;

        appliedEventCount++;
        lastSequence = queuedEvent.Sequence;
        lastLeg = queuedEvent.Leg;
        lastSourceTimestamp = queuedEvent.SourceTimestamp;
        lastUnityTimestamp = unityTimestamp;
        lastReferencePosition = referencePosition;

        AppliedPeakEvent appliedEvent = new AppliedPeakEvent(
            queuedEvent.Leg,
            queuedEvent.SourceTimestamp,
            unityTimestamp,
            queuedEvent.Sequence,
            referencePosition,
            receivedEventCount,
            appliedEventCount);

        try
        {
            PeakApplied?.Invoke(appliedEvent);
        }
        catch (Exception exception)
        {
            UnityEngine.Debug.LogException(exception, this);
        }

        if (logEvents)
        {
            UnityEngine.Debug.Log(
                $"[LegPeakEventReceiver] seq={queuedEvent.Sequence}, " +
                $"leg={queuedEvent.Leg}, sourceTime={queuedEvent.SourceTimestamp:F4}, " +
                $"unityTime={unityTimestamp:F4}, reference={referencePosition}",
                this);
        }
    }

    // ===== 单调时钟 =====
    // 当输入源没有提供时间戳时使用Stopwatch，避免系统时间调整造成时间倒退。

    private static double GetMonotonicSeconds()
    {
        return (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
    }

    // ===== 已应用事件的数据快照 =====
    // 同时保留源时间、Unity应用时间、序号和事件发生时的参考位置，
    // 供控制器计算周期/位移，也供CSV与诊断组件记录。

    public readonly struct AppliedPeakEvent
    {
        public AppliedPeakEvent(
            PeakLegSide leg,
            double sourceTimestamp,
            double unityTimestamp,
            int sequence,
            Vector3 referencePosition,
            int receivedCount,
            int appliedCount)
        {
            Leg = leg;
            SourceTimestamp = sourceTimestamp;
            UnityTimestamp = unityTimestamp;
            Sequence = sequence;
            ReferencePosition = referencePosition;
            ReceivedCount = receivedCount;
            AppliedCount = appliedCount;
        }

        public PeakLegSide Leg { get; }
        public double SourceTimestamp { get; }
        public double UnityTimestamp { get; }
        public int Sequence { get; }
        public Vector3 ReferencePosition { get; }
        public int ReceivedCount { get; }
        public int AppliedCount { get; }
    }

    // ===== 队列内部的最小事件格式 =====
    // 入队阶段不读取Transform；位置和Unity时间要等到主线程ApplyPeak时再采集。

    private readonly struct QueuedPeakEvent
    {
        public QueuedPeakEvent(
            PeakLegSide leg,
            double sourceTimestamp,
            int sequence)
        {
            Leg = leg;
            SourceTimestamp = sourceTimestamp;
            Sequence = sequence;
        }

        public PeakLegSide Leg { get; }
        public double SourceTimestamp { get; }
        public int Sequence { get; }
    }
}
