using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

/// <summary>
/// Sends loopback UDP messages for testing the real UDP path.
/// Q sends "left is up".
/// E sends "right is up".
/// Space pauses/resumes automatic alternating messages.
/// </summary>
public sealed class UdpPeakEventLoopbackSender : MonoBehaviour
{
    [Header("Destination")]
    [SerializeField] private string destinationAddress = "127.0.0.1";

    [Range(1024, 65535)]
    [SerializeField] private int destinationPort = 5055;

    [Header("Automatic Test")]
    [SerializeField] private bool automaticStream = false;

    [Min(0.1f)]
    [SerializeField] private float alternatingInterval = 0.50f;

    [SerializeField] private bool startWithLeft = true;

    [Min(0f)]
    [SerializeField] private float initialDelay = 0.50f;

    [Header("Manual Keys")]
    [SerializeField] private KeyCode leftKey = KeyCode.Q;
    [SerializeField] private KeyCode rightKey = KeyCode.E;
    [SerializeField] private KeyCode pauseKey = KeyCode.Space;

    [Header("Diagnostics")]
    [SerializeField] private int sentDatagramCount;
    [SerializeField] private string lastMessage = "";
    [SerializeField] private string lastError = "";

    private UdpClient udpClient;
    private IPEndPoint destination;
    private double nextSendTime;
    private bool nextIsLeft;

    private void Start()
    {
        try
        {
            destination = new IPEndPoint(
                IPAddress.Parse(destinationAddress),
                destinationPort);

            udpClient = new UdpClient();
            nextIsLeft = startWithLeft;
            nextSendTime =
                Time.realtimeSinceStartupAsDouble +
                Mathf.Max(0f, initialDelay);
        }
        catch (Exception exception)
        {
            lastError = exception.Message;
            Debug.LogException(exception, this);
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(leftKey))
        {
            SendMessageText("left is up");
        }

        if (Input.GetKeyDown(rightKey))
        {
            SendMessageText("right is up");
        }

        if (Input.GetKeyDown(pauseKey))
        {
            automaticStream = !automaticStream;
            nextSendTime =
                Time.realtimeSinceStartupAsDouble +
                alternatingInterval;
        }

        if (!automaticStream)
        {
            return;
        }

        double now = Time.realtimeSinceStartupAsDouble;

        if (now >= nextSendTime)
        {
            SendMessageText(
                nextIsLeft
                    ? "left is up"
                    : "right is up");

            nextIsLeft = !nextIsLeft;
            nextSendTime = now + alternatingInterval;
        }
    }

    [ContextMenu("Send Left")]
    public void SendLeft()
    {
        SendMessageText("left is up");
    }

    [ContextMenu("Send Right")]
    public void SendRight()
    {
        SendMessageText("right is up");
    }

    private void SendMessageText(string message)
    {
        if (udpClient == null || destination == null)
        {
            lastError = "UDP sender is not initialized.";
            return;
        }

        try
        {
            byte[] data = Encoding.UTF8.GetBytes(message);
            udpClient.Send(data, data.Length, destination);

            sentDatagramCount++;
            lastMessage = message;
            lastError = "";

            Debug.Log(
                $"[UdpPeakEventLoopbackSender] Sent " +
                $"\"{message}\" to {destination}.",
                this);
        }
        catch (Exception exception)
        {
            lastError = exception.Message;
            Debug.LogException(exception, this);
        }
    }

    private void OnDestroy()
    {
        udpClient?.Close();
        udpClient = null;
    }
}
