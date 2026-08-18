using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Receives raw 6-axis IMU data from N ESP32 boards over UDP and optionally drives one cube per
/// board. Fusion (complementary filter) runs here rather than on the boards so it can be tuned
/// without reflashing. See firmware/README.md for the wire format and setup steps.
///
/// One socket per board: each Pi bridge script owns a board and a port, so board identity comes
/// from WHICH PORT a packet arrived on rather than from the id field inside it. A board flashed
/// with the wrong BOARD_ID therefore cannot silently drive the other board's cube -- it gets a
/// warning instead.
///
/// This component is transport + fusion only. It never touches an Avatar, Foot Target, bone, or
/// IK target. Consumers subscribe to <see cref="SampleFused"/> (raised on the Unity main thread)
/// or poll <see cref="TryGetOrientation"/>.
/// </summary>
public class imu_data_receive : MonoBehaviour
{
    /// <summary>Board count used only to size the Inspector defaults. The live count is
    /// <see cref="BoardCount"/>, which follows <see cref="listenPorts"/>.</summary>
    public const int DefaultBoardCount = 2;

    [Header("UDP")]
    [Tooltip("One port per board. Element 0 = imu_serial_bridge_0.py, element 1 = imu_serial_bridge_1.py. " +
             "Adding an entry adds a board; no code change is needed.")]
    public int[] listenPorts = new int[DefaultBoardCount] { 5676, 5677 };

    [Header("Targets")]
    [Tooltip("Optional debug cubes. Element 0 is driven by board 0, element 1 by board 1. " +
             "Leave empty when this component is only feeding a detector.")]
    public Transform[] cubes = new Transform[DefaultBoardCount];

    [Header("Fusion")]
    [Range(0.5f, 0.999f)]
    [Tooltip("Higher = trust the gyro more: smoother, but drifts faster")]
    public float gyroTrustAlpha = 0.98f;
    [Range(0.01f, 0.5f)]
    [Tooltip("How close |accel| must be to 9.81 before it is treated as gravity")]
    public float gravityTolerance = 0.10f;

    [Header("Diagnostics (read-only)")]
    public int[] packetsPerSecond = new int[DefaultBoardCount];
    public int[] droppedPackets = new int[DefaultBoardCount];

    /// <summary>
    /// One fused sample, in the units the board sends: acceleration in m/s^2, angular velocity in
    /// rad/s. Raised on the Unity main thread, once per board per frame at most.
    /// </summary>
    public readonly struct ImuSampleSnapshot
    {
        public ImuSampleSnapshot(
            int board,
            uint sequence,
            uint timestampMicros,
            Vector3 acceleration,
            Vector3 angularVelocity,
            float deltaTime)
        {
            Board = board;
            Sequence = sequence;
            TimestampMicros = timestampMicros;
            Acceleration = acceleration;
            AngularVelocity = angularVelocity;
            DeltaTime = deltaTime;
        }

        public int Board { get; }
        public uint Sequence { get; }
        public uint TimestampMicros { get; }

        /// <summary>Raw accelerometer vector, m/s^2, board frame.</summary>
        public Vector3 Acceleration { get; }

        /// <summary>Raw gyroscope vector, rad/s, board frame.</summary>
        public Vector3 AngularVelocity { get; }

        /// <summary>Board-clock seconds since this board's previous fused sample.
        /// Zero when unknown (first sample) or rejected as bogus.</summary>
        public float DeltaTime { get; }

        public bool HasDeltaTime => DeltaTime > 0f;
    }

    /// <summary>
    /// Raised once per accepted sample, per board, from Update on the Unity main thread.
    /// Safe for subscribers to touch Unity API from.
    /// </summary>
    public event Action<ImuSampleSnapshot> SampleFused;

    /// <summary>Live board count. Follows <see cref="listenPorts"/> so a pelvis board can be
    /// added from the Inspector alone.</summary>
    public int BoardCount => listenPorts != null ? listenPorts.Length : 0;

    // Latest sample per board. Written on the receive thread, read on the main thread; both
    // guarded by _sampleLock. Deliberately plain floats -- Unity types must not be touched here.
    private struct ImuSample
    {
        public uint seq;
        public uint tMicros;
        public float ax, ay, az;
        public float gx, gy, gz;
        public bool valid;
    }

    private ImuSample[] _latest;
    private readonly object _sampleLock = new object();

    // Main-thread fusion state.
    private Quaternion[] _orientation;
    private Quaternion[] _recenterOffset;
    private uint[] _lastFusedMicros;
    private bool[] _hasFused;

    // Diagnostics counters, incremented on the receive thread.
    private int[] _packetCounter;
    private uint[] _lastSeq;
    private bool[] _hasSeq;
    private float _nextRateSample;

    // One socket + thread per board, indexed the same as everything else.
    private UdpClient[] _udpClients;
    private Thread[] _receiveThreads;
    private volatile bool _isRunning;

    // Logged at most once per board: a 100 Hz stream would flood the Console otherwise.
    private bool[] _idMismatchLogged;

    // Sized once in Start, before any receive thread exists, so the threads never race allocation.
    private int _activeBoardCount;

    void Start()
    {
        AllocateBoardState();
        StartUDPListener();
    }

    // All per-board arrays are sized here and never resized while sockets are open. Changing
    // listenPorts at runtime therefore has no effect until the next Play session -- deliberate,
    // since a resize under a live receive thread would be a data race.
    private void AllocateBoardState()
    {
        _activeBoardCount = BoardCount;

        if (_activeBoardCount <= 0)
        {
            Debug.LogError("[IMU] listenPorts is empty; nothing to listen on.");
        }

        _latest = new ImuSample[_activeBoardCount];
        _orientation = new Quaternion[_activeBoardCount];
        _recenterOffset = new Quaternion[_activeBoardCount];
        _lastFusedMicros = new uint[_activeBoardCount];
        _hasFused = new bool[_activeBoardCount];
        _packetCounter = new int[_activeBoardCount];
        _lastSeq = new uint[_activeBoardCount];
        _hasSeq = new bool[_activeBoardCount];
        _udpClients = new UdpClient[_activeBoardCount];
        _receiveThreads = new Thread[_activeBoardCount];
        _idMismatchLogged = new bool[_activeBoardCount];

        // Inspector arrays may have been authored at a different length; keep them in step so the
        // read-only diagnostics line up with the boards they describe.
        if (packetsPerSecond == null || packetsPerSecond.Length != _activeBoardCount)
        {
            packetsPerSecond = new int[_activeBoardCount];
        }

        if (droppedPackets == null || droppedPackets.Length != _activeBoardCount)
        {
            droppedPackets = new int[_activeBoardCount];
        }

        for (int i = 0; i < _activeBoardCount; i++)
        {
            _orientation[i] = Quaternion.identity;
            _recenterOffset[i] = Quaternion.identity;
        }
    }

    void StartUDPListener()
    {
        _isRunning = true;

        for (int i = 0; i < _activeBoardCount; i++)
        {
            // Each port binds independently: one board failing (port already in use, say) must
            // not stop the other from running.
            int boardIndex = i; // capture per-iteration, not the loop variable
            int port = listenPorts[i];
            try
            {
                _udpClients[boardIndex] = new UdpClient(port);
                _receiveThreads[boardIndex] = new Thread(() => ReceiveData(boardIndex)) { IsBackground = true };
                _receiveThreads[boardIndex].Start();
                Debug.Log($"[IMU] Board {boardIndex}: listening on UDP port {port}...");
            }
            catch (Exception e)
            {
                Debug.LogError($"[IMU] Board {boardIndex}: could not bind port {port}: {e.Message}");
            }
        }
    }

    private void ReceiveData(int boardIndex)
    {
        UdpClient client = _udpClients[boardIndex];
        IPEndPoint remote = new IPEndPoint(IPAddress.Any, listenPorts[boardIndex]);
        while (_isRunning)
        {
            try
            {
                byte[] data = client.Receive(ref remote);
                ParsePacket(Encoding.UTF8.GetString(data), boardIndex);
            }
            catch (SocketException se)
            {
                if (_isRunning) // sockets throw on Close(); that is expected during shutdown
                    Debug.LogError($"[IMU] Board {boardIndex} socket error: {se.Message}");
            }
            catch (Exception e)
            {
                if (_isRunning)
                    Debug.LogError($"[IMU] Board {boardIndex} receive error: {e.Message}");
            }
        }
    }

    // id,seq,t_us,ax,ay,az,gx,gy,gz
    //
    // boardIndex comes from the socket the packet arrived on, and is what the sample is filed
    // under. The id field is only a consistency check -- see the mismatch warning below.
    private void ParsePacket(string message, int boardIndex)
    {
        string[] p = message.Trim().Split(',');
        if (p.Length < 9) return;

        var inv = CultureInfo.InvariantCulture;
        int id;
        if (!int.TryParse(p[0], NumberStyles.Integer, inv, out id)) return;

        if (id != boardIndex && !_idMismatchLogged[boardIndex])
        {
            _idMismatchLogged[boardIndex] = true;
            Debug.LogWarning(
                $"[IMU] Port {listenPorts[boardIndex]} is carrying id={id}, but it is wired to " +
                $"board {boardIndex}. Routing by port, so this board drives cubes[{boardIndex}]. " +
                $"Reflash BOARD_ID or swap the USB sockets on the Pi if that is not what you want.");
        }

        ImuSample s;
        if (!uint.TryParse(p[1], NumberStyles.Integer, inv, out s.seq)) return;
        if (!uint.TryParse(p[2], NumberStyles.Integer, inv, out s.tMicros)) return;
        if (!float.TryParse(p[3], NumberStyles.Float, inv, out s.ax)) return;
        if (!float.TryParse(p[4], NumberStyles.Float, inv, out s.ay)) return;
        if (!float.TryParse(p[5], NumberStyles.Float, inv, out s.az)) return;
        if (!float.TryParse(p[6], NumberStyles.Float, inv, out s.gx)) return;
        if (!float.TryParse(p[7], NumberStyles.Float, inv, out s.gy)) return;
        if (!float.TryParse(p[8], NumberStyles.Float, inv, out s.gz)) return;
        s.valid = true;

        lock (_sampleLock)
        {
            _latest[boardIndex] = s;
            _packetCounter[boardIndex]++;
            if (_hasSeq[boardIndex] && s.seq > _lastSeq[boardIndex] + 1)
                droppedPackets[boardIndex] += (int)(s.seq - _lastSeq[boardIndex] - 1);
            else if (s.seq < _lastSeq[boardIndex])
                droppedPackets[boardIndex] = 0; // board rebooted
            _lastSeq[boardIndex] = s.seq;
            _hasSeq[boardIndex] = true;
        }
    }

    void Update()
    {
        for (int i = 0; i < _activeBoardCount; i++)
        {
            ImuSample s;
            lock (_sampleLock)
            {
                s = _latest[i];
                _latest[i].valid = false; // only fuse each sample once
            }
            if (!s.valid) continue;

            // Only the newest sample of the frame survives -- at 100 Hz into a 72 Hz render loop
            // some are dropped. Harmless for a ~1 Hz gait signal, which is still ~70x oversampled.
            float dt = Fuse(i, s);

            if (i < cubes.Length && cubes[i] != null)
                cubes[i].localRotation = _recenterOffset[i] * _orientation[i];

            SampleFused?.Invoke(
                new ImuSampleSnapshot(
                    i,
                    s.seq,
                    s.tMicros,
                    new Vector3(s.ax, s.ay, s.az),
                    new Vector3(s.gx, s.gy, s.gz),
                    dt));
        }

        if (Time.unscaledTime >= _nextRateSample)
        {
            lock (_sampleLock)
            {
                for (int i = 0; i < _activeBoardCount; i++)
                {
                    packetsPerSecond[i] = _packetCounter[i];
                    _packetCounter[i] = 0;
                }
            }
            _nextRateSample = Time.unscaledTime + 1f;
        }
    }

    /// <returns>The board-clock delta used for this sample, or 0 when it could not be trusted.</returns>
    private float Fuse(int i, ImuSample s)
    {
        if (!_hasFused[i])
        {
            _lastFusedMicros[i] = s.tMicros;
            _hasFused[i] = true;
            return 0f;
        }

        // Board-side timestamps, not Time.deltaTime -- the render rate and the sensor rate are
        // unrelated. Unsigned subtraction handles the micros() wrap at ~71 minutes for free.
        float dt = unchecked(s.tMicros - _lastFusedMicros[i]) / 1e6f;
        _lastFusedMicros[i] = s.tMicros;
        if (dt <= 0f || dt > 0.5f) return 0f; // stale or bogus interval

        // 1. Integrate the gyro (rad/s -> deg over dt).
        Vector3 gyroDeg = new Vector3(s.gx, s.gy, s.gz) * Mathf.Rad2Deg * dt;
        _orientation[i] *= Quaternion.Euler(gyroDeg);

        // 2. Only trust the accelerometer as a gravity reference when the board is near rest --
        //    while it is being moved, the vector is gravity + motion and blending it in lurches.
        Vector3 accel = new Vector3(s.ax, s.ay, s.az);
        float magnitude = accel.magnitude;
        if (Mathf.Abs(magnitude - 9.81f) > 9.81f * gravityTolerance) return dt;

        // 3. Slerp toward the orientation whose "up" matches the measured gravity direction.
        //    This corrects pitch and roll only; yaw has no reference on a 6-axis IMU.
        Vector3 measuredUp = _orientation[i] * accel.normalized;
        Quaternion correction = Quaternion.FromToRotation(measuredUp, Vector3.up) * _orientation[i];
        _orientation[i] = Quaternion.Slerp(_orientation[i], correction, 1f - gyroTrustAlpha);

        return dt;
    }

    /// <summary>
    /// Current recentred orientation for one board. Pitch and roll are gravity-corrected only
    /// while the board is near rest; yaw has no reference on a 6-axis IMU and drifts.
    /// </summary>
    public bool TryGetOrientation(int board, out Quaternion orientation)
    {
        if (_orientation == null ||
            board < 0 ||
            board >= _activeBoardCount ||
            !_hasFused[board])
        {
            orientation = Quaternion.identity;
            return false;
        }

        orientation = _recenterOffset[board] * _orientation[board];
        return true;
    }

    /// <summary>
    /// Zeroes every board against its current orientation. Hold the boards flat and still first.
    /// Use this to cancel accumulated yaw drift between trials. Safe to call at runtime.
    /// </summary>
    [ContextMenu("Recenter All")]
    public void RecenterAll()
    {
        if (_orientation == null) return;

        for (int i = 0; i < _activeBoardCount; i++)
            _recenterOffset[i] = Quaternion.Inverse(_orientation[i]);
        Debug.Log($"[IMU] Recentered {_activeBoardCount} board(s).");
    }

    /// <summary>Zeroes one board. Safe to call at runtime.</summary>
    public void RecenterBoard(int board)
    {
        if (_orientation == null || board < 0 || board >= _activeBoardCount) return;

        _recenterOffset[board] = Quaternion.Inverse(_orientation[board]);
        Debug.Log($"[IMU] Recentered board {board}.");
    }

    void OnDisable()
    {
        StopUDPListener();
    }

    void OnApplicationQuit()
    {
        StopUDPListener();
    }

    void StopUDPListener()
    {
        if (!_isRunning) return;
        _isRunning = false;

        for (int i = 0; i < _activeBoardCount; i++)
        {
            // Close first so the blocking Receive() throws and the thread can exit.
            _udpClients[i]?.Close();
            _udpClients[i] = null;
            _receiveThreads[i]?.Join(500);
            _receiveThreads[i] = null;
        }
        Debug.Log("[IMU] Stopped listening.");
    }
}
