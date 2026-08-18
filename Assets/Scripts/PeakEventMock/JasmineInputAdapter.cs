using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Internal semantic event types understood by the Jasmine adapter.
/// These names do not assume that the external Jasmine protocol uses
/// the same strings.
/// </summary>
public enum JasmineLegEventType
{
    Unknown,
    Top,
    HitBottom,
    StartGoingUp
}

/// <summary>
/// Isolated input boundary for Jasmine belt integration.
///
/// Two producers feed the same semantic pipeline:
///   raw UDP    -> thread-safe queue -> parse/validate -\
///                                                       >- semantic event
///   local IMU  -> SubmitSemanticEvent ------------------/
///        -> duplicate filter -> mapTopToPeak gate -> LegPeakEventReceiver.
///
/// Local events are routed through here rather than straight to SubmitPeak so that both
/// producers share one set of filters, counters, and safety gates.
///
/// This component never moves an Avatar, Foot Target, bone, or IK target,
/// and it does not read or modify the gait controller state machine.
/// </summary>
public sealed class JasmineInputAdapter : MonoBehaviour
{
    private const int ExistingPeakReceiverPort = 5055;

    [Header("Output")]
    [SerializeField] private LegPeakEventReceiver peakEventReceiver;

    [Header("Safety")]
    [Tooltip("Master switch. When false, this component does not bind a UDP socket.")]
    [SerializeField] private bool enableJasmineInput = false;

    [Tooltip("Only confirmed Top events may be forwarded to SubmitPeak. Disabled by default.")]
    [SerializeField] private bool mapTopToPeak = false;

    [Header("Jasmine UDP")]
    [Tooltip("Dedicated Jasmine port. Port 5055 is reserved for the existing Peak receiver.")]
    [Range(1024, 65535)]
    [SerializeField] private int listenPort = 5056;

    [Header("Logging")]
    [SerializeField] private bool logRawMessages = true;
    [SerializeField] private bool logParsedEvents = true;

    [Tooltip("When enabled, unknown messages produce a parse warning. They never drive gait.")]
    [SerializeField] private bool acceptUnknownMessages = true;

    [Header("Diagnostics")]
    [SerializeField] private bool isListening;
    [SerializeField] private bool receiveThreadAlive;
    [SerializeField] private int receivedDatagramCount;
    [SerializeField] private int parsedEventCount;
    [SerializeField] private int forwardedTopCount;
    [SerializeField] private int unknownMessageCount;
    [SerializeField] private int invalidEncodingCount;
    [SerializeField] private int duplicateOrOutOfOrderCount;
    [SerializeField] private string boundEndpoint = "";
    [SerializeField] private string lastSourceIp = "";
    [SerializeField] private string lastRawMessage = "";
    [SerializeField] private string lastParsedEvent = "";
    [SerializeField] private string lastWarning = "";
    [SerializeField] private string lastError = "";

    private readonly ConcurrentQueue<ReceivedDatagram> receivedDatagrams =
        new ConcurrentQueue<ReceivedDatagram>();

    private readonly ConcurrentQueue<ThreadNotice> threadNotices =
        new ConcurrentQueue<ThreadNotice>();

    private readonly UTF8Encoding strictUtf8 =
        new UTF8Encoding(false, true);

    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool stopRequested;
    private int localReceiveSequence;
    private int lastLeftSourceSequence = -1;
    private int lastRightSourceSequence = -1;

    public bool IsListening => isListening;
    public int ReceivedDatagramCount => receivedDatagramCount;
    public int ParsedEventCount => parsedEventCount;
    public int ForwardedTopCount => forwardedTopCount;

    /// <summary>Whether confirmed Top events reach the gait controller. Exposed so a local
    /// detector can report "detecting but not driving" instead of appearing broken.</summary>
    public bool MapTopToPeak => mapTopToPeak;

    private void Awake()
    {
        if (peakEventReceiver == null)
        {
            peakEventReceiver = FindFirstObjectByType<LegPeakEventReceiver>();
        }
    }

    private void OnEnable()
    {
        // The component may remain enabled for Inspector diagnostics while the
        // master switch keeps the socket completely inactive.
        if (enableJasmineInput)
        {
            StartListening();
        }
    }

    private void Update()
    {
        receiveThreadAlive =
            receiveThread != null &&
            receiveThread.IsAlive;

        while (receivedDatagrams.TryDequeue(
                   out ReceivedDatagram datagram))
        {
            ProcessDatagram(datagram);
        }

        while (threadNotices.TryDequeue(
                   out ThreadNotice notice))
        {
            ApplyThreadNotice(notice);
        }
    }

    [ContextMenu("Start Jasmine Listening")]
    public void StartListening()
    {
        if (isListening)
        {
            return;
        }

        if (!enableJasmineInput)
        {
            lastWarning =
                "Jasmine input is disabled. Enable it in the Inspector first.";

            UnityEngine.Debug.LogWarning(
                $"[JasmineInputAdapter] {lastWarning}",
                this);

            return;
        }

        // Refuse the existing Peak port even if it is entered accidentally.
        // This preserves the original 1/0 and left/right receiver.
        if (listenPort == ExistingPeakReceiverPort)
        {
            lastError =
                "UDP port 5055 is reserved for UdpPeakEventReceiver. " +
                "Choose a separate Jasmine port.";

            UnityEngine.Debug.LogError(
                $"[JasmineInputAdapter] {lastError}",
                this);

            return;
        }

        try
        {
            stopRequested = false;
            lastError = "";

            udpClient =
                new UdpClient(AddressFamily.InterNetwork);

            udpClient.Client.ExclusiveAddressUse = true;
            udpClient.Client.Bind(
                new IPEndPoint(IPAddress.Any, listenPort));

            udpClient.Client.ReceiveTimeout = 500;
            boundEndpoint =
                udpClient.Client.LocalEndPoint?.ToString() ??
                $"0.0.0.0:{listenPort}";

            receiveThread =
                new Thread(ReceiveLoop)
                {
                    IsBackground = true,
                    Name = $"JasmineInputAdapter:{listenPort}"
                };

            isListening = true;
            receiveThread.Start();

            UnityEngine.Debug.Log(
                $"[JasmineInputAdapter] Listening on {boundEndpoint}. " +
                "Raw-log mode is ready; no protocol is assumed.",
                this);
        }
        catch (Exception exception)
        {
            isListening = false;
            lastError = exception.Message;

            UnityEngine.Debug.LogException(
                exception,
                this);

            CloseSocket();
        }
    }

    [ContextMenu("Stop Jasmine Listening")]
    public void StopListening()
    {
        stopRequested = true;
        isListening = false;

        CloseSocket();

        if (receiveThread != null &&
            receiveThread.IsAlive &&
            Thread.CurrentThread != receiveThread)
        {
            receiveThread.Join(1000);
        }

        receiveThread = null;
        receiveThreadAlive = false;
    }

    [ContextMenu("Reset Jasmine Diagnostics")]
    public void ResetDiagnostics()
    {
        receivedDatagramCount = 0;
        parsedEventCount = 0;
        forwardedTopCount = 0;
        unknownMessageCount = 0;
        invalidEncodingCount = 0;
        duplicateOrOutOfOrderCount = 0;
        localReceiveSequence = 0;
        lastLeftSourceSequence = -1;
        lastRightSourceSequence = -1;
        lastSourceIp = "";
        lastRawMessage = "";
        lastParsedEvent = "";
        lastWarning = "";
        lastError = "";

        while (receivedDatagrams.TryDequeue(out _))
        {
        }

        while (threadNotices.TryDequeue(out _))
        {
        }
    }

    private void ReceiveLoop()
    {
        IPEndPoint remoteEndpoint =
            new IPEndPoint(IPAddress.Any, 0);

        while (!stopRequested)
        {
            try
            {
                byte[] data =
                    udpClient.Receive(ref remoteEndpoint);

                double receivedTime =
                    GetMonotonicSeconds();

                int receiveSequence =
                    Interlocked.Increment(
                        ref localReceiveSequence) - 1;

                Interlocked.Increment(
                    ref receivedDatagramCount);

                string rawMessage;
                bool validUtf8 = true;

                try
                {
                    rawMessage =
                        strictUtf8.GetString(data);
                }
                catch (DecoderFallbackException)
                {
                    validUtf8 = false;

                    Interlocked.Increment(
                        ref invalidEncodingCount);

                    // Preserve invalid payloads in a reversible text form.
                    rawMessage =
                        "<invalid-utf8 base64=" +
                        Convert.ToBase64String(data) +
                        ">";
                }

                receivedDatagrams.Enqueue(
                    new ReceivedDatagram(
                        rawMessage,
                        remoteEndpoint.ToString(),
                        receivedTime,
                        receiveSequence,
                        validUtf8));
            }
            catch (SocketException socketException)
            {
                if (stopRequested)
                {
                    break;
                }

                if (socketException.SocketErrorCode ==
                    SocketError.TimedOut)
                {
                    continue;
                }

                threadNotices.Enqueue(
                    ThreadNotice.Error(
                        socketException.Message));
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception exception)
            {
                if (stopRequested)
                {
                    break;
                }

                threadNotices.Enqueue(
                    ThreadNotice.Error(
                        exception.Message));
            }
        }
    }

    private void ProcessDatagram(
        ReceivedDatagram datagram)
    {
        lastSourceIp = datagram.SourceEndpoint;
        lastRawMessage = datagram.RawMessage;

        if (logRawMessages)
        {
            UnityEngine.Debug.Log(
                "[JasmineRaw] " +
                $"message=\"{EscapeForLog(datagram.RawMessage)}\" " +
                $"receivedTime={datagram.ReceivedTime:F6} " +
                $"sourceIP={datagram.SourceEndpoint}",
                this);
        }

        if (!datagram.ValidUtf8)
        {
            WarnAndIgnore(
                "Invalid UTF-8 payload; retained as Base64.");

            return;
        }

        if (string.IsNullOrWhiteSpace(
                datagram.RawMessage))
        {
            WarnAndIgnore("Empty message.");
            return;
        }

        if (!TryParseMessage(
                datagram,
                out JasmineLegEvent parsedEvent,
                out string parseWarning))
        {
            unknownMessageCount++;

            if (acceptUnknownMessages)
            {
                WarnAndIgnore(parseWarning);
            }

            return;
        }

        DispatchSemanticEvent(parsedEvent);
    }

    /// <summary>
    /// Entry point for a semantic leg event produced locally rather than received over UDP --
    /// currently <c>ImuGaitEventDetector</c>, which derives Top/HitBottom/StartGoingUp from the
    /// hip-pocket IMUs.
    ///
    /// Local events deliberately share this component's filters, counters, and the
    /// <c>mapTopToPeak</c> gate, so a detector cannot become a second undocumented gait source
    /// that bypasses them.
    ///
    /// Independent of <c>enableJasmineInput</c> and <c>isListening</c>: those govern the UDP
    /// socket only, and a local detector needs no socket. Main thread only.
    /// </summary>
    public void SubmitSemanticEvent(
        PeakLegSide leg,
        JasmineLegEventType eventType,
        double sourceTimestamp,
        int sequence)
    {
        // Unlike the placeholder UDP grammar, a local detector genuinely owns both a monotonic
        // clock and a monotonic sequence, so both are flagged as source-provided. That is what
        // arms IsDuplicateOrOutOfOrder for this path.
        DispatchSemanticEvent(
            new JasmineLegEvent(
                leg,
                eventType,
                sourceTimestamp,
                true,
                sequence,
                true,
                "<local-detector>"));
    }

    // Shared tail of both paths: duplicate rejection, counters, logging, and the per-type
    // handlers. Everything above this point is transport-specific.
    private void DispatchSemanticEvent(
        JasmineLegEvent parsedEvent)
    {
        if (IsDuplicateOrOutOfOrder(parsedEvent))
        {
            duplicateOrOutOfOrderCount++;

            WarnAndIgnore(
                "Duplicate or out-of-order source sequence.");

            return;
        }

        parsedEventCount++;
        lastParsedEvent =
            $"{parsedEvent.Leg}/{parsedEvent.EventType}";

        if (logParsedEvents)
        {
            UnityEngine.Debug.Log(
                "[JasmineEvent] " +
                $"Leg={parsedEvent.Leg} " +
                $"Type={parsedEvent.EventType} " +
                $"Timestamp={parsedEvent.SourceTimestamp:F6} " +
                $"HasSourceTimestamp={parsedEvent.HasSourceTimestamp} " +
                $"Sequence={parsedEvent.Sequence} " +
                $"HasSourceSequence={parsedEvent.HasSourceSequence}",
                this);
        }

        switch (parsedEvent.EventType)
        {
            case JasmineLegEventType.Top:
                HandleTop(parsedEvent);
                break;

            case JasmineLegEventType.HitBottom:
                HandleHitBottom(parsedEvent);
                break;

            case JasmineLegEventType.StartGoingUp:
                HandleStartGoingUp(parsedEvent);
                break;

            default:
                WarnAndIgnore(
                    "Parsed event type is Unknown.");
                break;
        }
    }

    /// <summary>
    /// Protocol extension point.
    ///
    /// Current provisional diagnostic grammar is intentionally narrow:
    /// "left top", "right top",
    /// "left hitbottom", "right hitbottom",
    /// "left startgoingup", "right startgoingup".
    ///
    /// It exists only to exercise the semantic pipeline. Tomorrow's real
    /// protocol must be implemented here after observing JasmineRaw logs.
    /// No timestamp or sequence syntax is guessed at this stage.
    /// </summary>
    private bool TryParseMessage(
        ReceivedDatagram datagram,
        out JasmineLegEvent parsedEvent,
        out string warning)
    {
        parsedEvent = default;
        warning = "";

        string normalized =
            Normalize(datagram.RawMessage);

        string[] tokens =
            normalized.Split(
                new[] { ' ' },
                StringSplitOptions.RemoveEmptyEntries);

        if (tokens.Length != 2)
        {
            warning =
                "Unknown format. Raw message was retained; " +
                "no gait event was generated.";

            return false;
        }

        if (!TryParseLeg(
                tokens[0],
                out PeakLegSide leg))
        {
            warning =
                "Missing or unknown leg. " +
                "No gait event was generated.";

            return false;
        }

        if (!TryParseEventType(
                tokens[1],
                out JasmineLegEventType eventType))
        {
            warning =
                "Missing or unknown event type. " +
                "No gait event was generated.";

            return false;
        }

        // The current placeholder grammar has no real source timestamp
        // or source sequence. Reception values are retained as fallbacks,
        // with explicit flags showing that they were not source-provided.
        parsedEvent =
            new JasmineLegEvent(
                leg,
                eventType,
                datagram.ReceivedTime,
                false,
                datagram.ReceiveSequence,
                false,
                datagram.RawMessage);

        return true;
    }

    private void HandleTop(
        JasmineLegEvent jasmineEvent)
    {
        if (!mapTopToPeak)
        {
            if (logParsedEvents)
            {
                UnityEngine.Debug.Log(
                    "[JasmineEvent] Top parsed but " +
                    "mapTopToPeak is disabled; no Peak forwarded.",
                    this);
            }

            return;
        }

        if (peakEventReceiver == null)
        {
            WarnAndIgnore(
                "Top mapping is enabled but " +
                "LegPeakEventReceiver is not assigned.");

            return;
        }

        peakEventReceiver.SubmitPeak(
            jasmineEvent.Leg,
            jasmineEvent.SourceTimestamp,
            jasmineEvent.Sequence);

        forwardedTopCount++;
    }

    private void HandleHitBottom(
        JasmineLegEvent jasmineEvent)
    {
        // Intentionally observation-only. LegPeakEventReceiver currently
        // represents Peak/Top events, not bottom-contact semantics.
        if (logParsedEvents)
        {
            UnityEngine.Debug.Log(
                $"[JasmineEvent] {jasmineEvent.Leg} HitBottom " +
                "recorded only; no Peak forwarded.",
                this);
        }
    }

    private void HandleStartGoingUp(
        JasmineLegEvent jasmineEvent)
    {
        // Intentionally observation-only until the protocol and desired
        // controller semantics are confirmed.
        if (logParsedEvents)
        {
            UnityEngine.Debug.Log(
                $"[JasmineEvent] {jasmineEvent.Leg} StartGoingUp " +
                "recorded only; no Peak forwarded.",
                this);
        }
    }

    private bool IsDuplicateOrOutOfOrder(
        JasmineLegEvent jasmineEvent)
    {
        if (!jasmineEvent.HasSourceSequence)
        {
            return false;
        }

        int lastSequence =
            jasmineEvent.Leg == PeakLegSide.Left
                ? lastLeftSourceSequence
                : lastRightSourceSequence;

        if (jasmineEvent.Sequence <= lastSequence)
        {
            return true;
        }

        if (jasmineEvent.Leg == PeakLegSide.Left)
        {
            lastLeftSourceSequence =
                jasmineEvent.Sequence;
        }
        else
        {
            lastRightSourceSequence =
                jasmineEvent.Sequence;
        }

        return false;
    }

    private void WarnAndIgnore(string warning)
    {
        lastWarning = warning;

        UnityEngine.Debug.LogWarning(
            $"[JasmineParseWarning] {warning}",
            this);
    }

    private void ApplyThreadNotice(
        ThreadNotice notice)
    {
        lastError = notice.Message;

        UnityEngine.Debug.LogError(
            $"[JasmineInputAdapter] {notice.Message}",
            this);
    }

    private static bool TryParseLeg(
        string token,
        out PeakLegSide leg)
    {
        if (token == "left")
        {
            leg = PeakLegSide.Left;
            return true;
        }

        if (token == "right")
        {
            leg = PeakLegSide.Right;
            return true;
        }

        leg = PeakLegSide.Left;
        return false;
    }

    private static bool TryParseEventType(
        string token,
        out JasmineLegEventType eventType)
    {
        switch (token)
        {
            case "top":
                eventType =
                    JasmineLegEventType.Top;
                return true;

            case "hitbottom":
                eventType =
                    JasmineLegEventType.HitBottom;
                return true;

            case "startgoingup":
                eventType =
                    JasmineLegEventType.StartGoingUp;
                return true;

            default:
                eventType =
                    JasmineLegEventType.Unknown;
                return false;
        }
    }

    private static string Normalize(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return "";
        }

        return value
            .Trim()
            .ToLowerInvariant()
            .Replace('-', ' ')
            .Replace('_', ' ')
            .Replace('\t', ' ');
    }

    private static string EscapeForLog(
        string value)
    {
        if (string.IsNullOrEmpty(value))
        {
            return "";
        }

        return value
            .Replace("\\", "\\\\")
            .Replace("\"", "\\\"")
            .Replace("\r", "\\r")
            .Replace("\n", "\\n");
    }

    private static double GetMonotonicSeconds()
    {
        return (double)Stopwatch.GetTimestamp() /
            Stopwatch.Frequency;
    }

    private void CloseSocket()
    {
        try
        {
            udpClient?.Close();
        }
        catch
        {
        }
        finally
        {
            udpClient = null;
        }
    }

    private void OnDisable()
    {
        StopListening();
    }

    private void OnApplicationQuit()
    {
        StopListening();
    }

    private readonly struct ReceivedDatagram
    {
        public ReceivedDatagram(
            string rawMessage,
            string sourceEndpoint,
            double receivedTime,
            int receiveSequence,
            bool validUtf8)
        {
            RawMessage = rawMessage;
            SourceEndpoint = sourceEndpoint;
            ReceivedTime = receivedTime;
            ReceiveSequence = receiveSequence;
            ValidUtf8 = validUtf8;
        }

        public string RawMessage { get; }
        public string SourceEndpoint { get; }
        public double ReceivedTime { get; }
        public int ReceiveSequence { get; }
        public bool ValidUtf8 { get; }
    }

    private readonly struct JasmineLegEvent
    {
        public JasmineLegEvent(
            PeakLegSide leg,
            JasmineLegEventType eventType,
            double sourceTimestamp,
            bool hasSourceTimestamp,
            int sequence,
            bool hasSourceSequence,
            string rawMessage)
        {
            Leg = leg;
            EventType = eventType;
            SourceTimestamp = sourceTimestamp;
            HasSourceTimestamp = hasSourceTimestamp;
            Sequence = sequence;
            HasSourceSequence = hasSourceSequence;
            RawMessage = rawMessage;
        }

        public PeakLegSide Leg { get; }
        public JasmineLegEventType EventType { get; }
        public double SourceTimestamp { get; }
        public bool HasSourceTimestamp { get; }
        public int Sequence { get; }
        public bool HasSourceSequence { get; }
        public string RawMessage { get; }
    }

    private readonly struct ThreadNotice
    {
        private ThreadNotice(string message)
        {
            Message = message;
        }

        public string Message { get; }

        public static ThreadNotice Error(
            string message)
        {
            return new ThreadNotice(message);
        }
    }
}
