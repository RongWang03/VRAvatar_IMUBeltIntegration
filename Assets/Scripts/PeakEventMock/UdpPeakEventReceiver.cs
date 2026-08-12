using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Receives sparse leg-peak events over UDP.
/// Supported messages:
///   left is up
///   right is up
/// Optional aliases:
///   left
///   right
///   left_up
///   right_up
///
/// Network receive runs on a background thread.
/// Unity logging and Inspector-facing string updates run on the main thread.
/// </summary>
public sealed class UdpPeakEventReceiver : MonoBehaviour
{
    [Header("Output")]
    [SerializeField] private LegPeakEventReceiver peakEventReceiver;

    [Header("UDP")]
    [Range(1024, 65535)]
    [SerializeField] private int listenPort = 5055;

    [SerializeField] private bool startOnEnable = true;

    [Tooltip("Accept aliases such as left, right, left_up, and right_up.")]
    [SerializeField] private bool acceptAliases = true;

    [Tooltip("Print valid and invalid messages in the Unity Console.")]
    [SerializeField] private bool logMessages = true;

    [Header("Diagnostics")]
    [SerializeField] private bool isListening;
    [SerializeField] private int receivedDatagramCount;
    [SerializeField] private int acceptedEventCount;
    [SerializeField] private int rejectedMessageCount;
    [SerializeField] private int localSequence;
    [SerializeField] private string lastRemoteEndpoint = "";
    [SerializeField] private string lastRawMessage = "";
    [SerializeField] private string lastParsedEvent = "";
    [SerializeField] private string lastError = "";
    [SerializeField] private string boundEndpoint = "";
    [SerializeField] private bool receiveThreadAlive;
    [SerializeField] private string listenerStartedUtc = "";
    [SerializeField] private string lastDatagramReceivedUtc = "";
    [SerializeField] private string listenerStoppedUtc = "";

    private readonly ConcurrentQueue<MainThreadUpdate> mainThreadUpdates =
        new ConcurrentQueue<MainThreadUpdate>();
    private readonly ConcurrentQueue<PendingPeakEvent> pendingPeakEvents =
        new ConcurrentQueue<PendingPeakEvent>();
    private readonly ConcurrentQueue<string> receiveTimeUpdates =
        new ConcurrentQueue<string>();

    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool stopRequested;

    public bool IsListening => isListening;
    public int ReceivedDatagramCount => receivedDatagramCount;
    public int AcceptedEventCount => acceptedEventCount;
    public int RejectedMessageCount => rejectedMessageCount;

    private void Awake()
    {
        if (peakEventReceiver == null)
        {
            peakEventReceiver = FindFirstObjectByType<LegPeakEventReceiver>();
        }
    }

    private void OnEnable()
    {
        if (startOnEnable)
        {
            StartListening();
        }
    }

    private void Update()
    {
        receiveThreadAlive = receiveThread != null && receiveThread.IsAlive;

        while (receiveTimeUpdates.TryDequeue(out string receivedUtc))
        {
            lastDatagramReceivedUtc = receivedUtc;
        }

        while (pendingPeakEvents.TryDequeue(out PendingPeakEvent pendingPeak))
        {
            peakEventReceiver.SubmitPeak(
                pendingPeak.Leg,
                sourceTimestamp: pendingPeak.SourceTimestamp,
                sequence: pendingPeak.Sequence);
        }

        while (mainThreadUpdates.TryDequeue(out MainThreadUpdate update))
        {
            lastRemoteEndpoint = update.RemoteEndpoint;
            lastRawMessage = update.RawMessage;
            lastParsedEvent = update.ParsedEvent;

            if (!string.IsNullOrEmpty(update.ErrorText))
            {
                lastError = update.ErrorText;
            }

            switch (update.LogLevel)
            {
                case MainThreadLogLevel.Info:
                    if (logMessages)
                    {
                        Debug.Log(update.LogMessage, this);
                    }
                    break;

                case MainThreadLogLevel.Warning:
                    if (logMessages)
                    {
                        Debug.LogWarning(update.LogMessage, this);
                    }
                    break;

                case MainThreadLogLevel.Error:
                    Debug.LogError(update.LogMessage, this);
                    break;
            }
        }
    }

    [ContextMenu("Start Listening")]
    public void StartListening()
    {
        if (isListening)
        {
            return;
        }

        if (peakEventReceiver == null)
        {
            lastError = "LegPeakEventReceiver is not assigned.";
            Debug.LogError($"[UdpPeakEventReceiver] {lastError}", this);
            return;
        }

        try
        {
            stopRequested = false;
            lastError = "";

            udpClient = new UdpClient(AddressFamily.InterNetwork);
            udpClient.Client.ExclusiveAddressUse = true;
            udpClient.Client.Bind(new IPEndPoint(IPAddress.Any, listenPort));
            udpClient.Client.ReceiveTimeout = 500;
            boundEndpoint = udpClient.Client.LocalEndPoint?.ToString() ?? $"0.0.0.0:{listenPort}";
            listenerStartedUtc = DateTime.UtcNow.ToString("O");
            listenerStoppedUtc = "";
            udpClient.Client.ReceiveTimeout = 500;

            receiveThread = new Thread(ReceiveLoop)
            {
                IsBackground = true,
                Name = $"UdpPeakEventReceiver:{listenPort}"
            };

            isListening = true;
            receiveThread.Start();

            Debug.Log($"[UdpPeakEventReceiver] StartListening succeeded. bound={boundEndpoint}, thread={receiveThread.Name}, object={name}, scene={gameObject.scene.path}", this);
            Debug.Log(
                $"[UdpPeakEventReceiver] Listening on UDP port {listenPort}.",
                this);
        }
        catch (Exception exception)
        {
            isListening = false;
            lastError = exception.Message;
            Debug.LogException(exception, this);
            CloseSocket();
        }
    }

    [ContextMenu("Stop Listening")]
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
        listenerStoppedUtc = DateTime.UtcNow.ToString("O");
        Debug.Log($"[UdpPeakEventReceiver] Listener stopped. bound={boundEndpoint}, received={receivedDatagramCount}, accepted={acceptedEventCount}, rejected={rejectedMessageCount}.", this);
    }

    [ContextMenu("Reset Diagnostics")]
    public void ResetDiagnostics()
    {
        receivedDatagramCount = 0;
        acceptedEventCount = 0;
        rejectedMessageCount = 0;
        localSequence = 0;
        lastRemoteEndpoint = "";
        lastRawMessage = "";
        lastParsedEvent = "";
        lastError = "";
        boundEndpoint = "";
        receiveThreadAlive = false;
        listenerStartedUtc = "";
        lastDatagramReceivedUtc = "";
        listenerStoppedUtc = "";

        while (mainThreadUpdates.TryDequeue(out _))
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
                byte[] data = udpClient.Receive(ref remoteEndpoint);
                string rawMessage = Encoding.UTF8.GetString(data);

                receiveTimeUpdates.Enqueue(DateTime.UtcNow.ToString("O"));
                Interlocked.Increment(ref receivedDatagramCount);

                if (TryParseLeg(rawMessage, out PeakLegSide leg))
                {
                    int sequence =
                        Interlocked.Increment(ref localSequence) - 1;

                    pendingPeakEvents.Enqueue(
                        new PendingPeakEvent(
                            leg,
                            GetMonotonicSeconds(),
                            sequence));

                    Interlocked.Increment(ref acceptedEventCount);

                    mainThreadUpdates.Enqueue(
                        MainThreadUpdate.CreateInfo(
                            remoteEndpoint.ToString(),
                            rawMessage,
                            leg.ToString(),
                            $"[UdpPeakEventReceiver] seq={sequence}, " +
                            $"leg={leg}, remote={remoteEndpoint}, " +
                            $"message=\"{rawMessage.Trim()}\""));
                }
                else
                {
                    Interlocked.Increment(ref rejectedMessageCount);

                    mainThreadUpdates.Enqueue(
                        MainThreadUpdate.CreateWarning(
                            remoteEndpoint.ToString(),
                            rawMessage,
                            "Rejected",
                            $"[UdpPeakEventReceiver] Rejected message: " +
                            $"\"{rawMessage.Trim()}\""));
                }
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

                mainThreadUpdates.Enqueue(
                    MainThreadUpdate.CreateError(
                        socketException.Message,
                        $"[UdpPeakEventReceiver] {socketException}"));
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

                mainThreadUpdates.Enqueue(
                    MainThreadUpdate.CreateError(
                        exception.Message,
                        $"[UdpPeakEventReceiver] {exception}"));
            }
        }

        isListening = false;
    }

    private bool TryParseLeg(
    string rawMessage,
    out PeakLegSide leg)
    {
        string message = Normalize(rawMessage);

        // New numeric messages.
        // 1 = left peak
        // 0 = right peak
        if (message == "1")
        {
            leg = PeakLegSide.Left;
            return true;
        }

        if (message == "0")
        {
            leg = PeakLegSide.Right;
            return true;
        }

        // Keep the original messages for compatibility.
        if (message == "left is up")
        {
            leg = PeakLegSide.Left;
            return true;
        }

        if (message == "right is up")
        {
            leg = PeakLegSide.Right;
            return true;
        }

        if (acceptAliases)
        {
            if (message == "left" ||
                message == "left up" ||
                message == "left_up")
            {
                leg = PeakLegSide.Left;
                return true;
            }

            if (message == "right" ||
                message == "right up" ||
                message == "right_up")
            {
                leg = PeakLegSide.Right;
                return true;
            }
        }

        leg = PeakLegSide.Left;
        return false;
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
            .Replace('\t', ' ');
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

    private enum MainThreadLogLevel
    {
        None,
        Info,
        Warning,
        Error
    }

    private static double GetMonotonicSeconds()
    {
        return (double)System.Diagnostics.Stopwatch.GetTimestamp() /
            System.Diagnostics.Stopwatch.Frequency;
    }

    private readonly struct PendingPeakEvent
    {
        public PendingPeakEvent(PeakLegSide leg, double sourceTimestamp, int sequence)
        {
            Leg = leg;
            SourceTimestamp = sourceTimestamp;
            Sequence = sequence;
        }

        public PeakLegSide Leg { get; }
        public double SourceTimestamp { get; }
        public int Sequence { get; }
    }

    private readonly struct MainThreadUpdate
    {
        private MainThreadUpdate(
            string remoteEndpoint,
            string rawMessage,
            string parsedEvent,
            string errorText,
            string logMessage,
            MainThreadLogLevel logLevel)
        {
            RemoteEndpoint = remoteEndpoint;
            RawMessage = rawMessage;
            ParsedEvent = parsedEvent;
            ErrorText = errorText;
            LogMessage = logMessage;
            LogLevel = logLevel;
        }

        public string RemoteEndpoint { get; }
        public string RawMessage { get; }
        public string ParsedEvent { get; }
        public string ErrorText { get; }
        public string LogMessage { get; }
        public MainThreadLogLevel LogLevel { get; }

        public static MainThreadUpdate CreateInfo(
            string remoteEndpoint,
            string rawMessage,
            string parsedEvent,
            string logMessage)
        {
            return new MainThreadUpdate(
                remoteEndpoint,
                rawMessage,
                parsedEvent,
                "",
                logMessage,
                MainThreadLogLevel.Info);
        }

        public static MainThreadUpdate CreateWarning(
            string remoteEndpoint,
            string rawMessage,
            string parsedEvent,
            string logMessage)
        {
            return new MainThreadUpdate(
                remoteEndpoint,
                rawMessage,
                parsedEvent,
                "",
                logMessage,
                MainThreadLogLevel.Warning);
        }

        public static MainThreadUpdate CreateError(
            string errorText,
            string logMessage)
        {
            return new MainThreadUpdate(
                "",
                "",
                "",
                errorText,
                logMessage,
                MainThreadLogLevel.Error);
        }
    }
}