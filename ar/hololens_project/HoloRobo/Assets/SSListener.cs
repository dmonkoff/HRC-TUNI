using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Scripts;
using System;
using System.Linq;
#if !UNITY_EDITOR && UNITY_METRO
    using Windows.Networking;
    using Windows.Networking.Sockets;
    using Windows.Storage.Streams;
    using Windows.Foundation;
    using System.Threading.Tasks;
#endif

#if UNITY_EDITOR
    using System.Text;
    using System.Threading;
    using System.IO;
#endif



public class SSListener : Singleton<SSListener> {


    [Tooltip("IP-Adress of Server.")]
    public string ServerIP = "130.230.155.89";

    [Tooltip("Port-Nmber used to connect to Server.")]
    public string ServerPort = "9001";

    [Tooltip("Number of bytes ")]
    public uint nBytes = 4096;

    public Text ButtonText = null;
    public string latestRecievedMsg;
    private bool IsSending = false;
    private bool IsReading = false;
    private bool isConnecting = false;
    public bool IsConnected { get; private set; }

#if UNITY_EDITOR
    System.Net.Sockets.TcpClient client;
    System.Net.Sockets.NetworkStream stream;
    private Thread exchangeThread;
    private StreamWriter writer;
    private StreamReader reader;
    private bool exchangeStopRequested = false;
#endif

#if !UNITY_EDITOR && UNITY_METRO
    private StreamSocket socket = null;
    private Task ListenerTask;
    private System.Text.ASCIIEncoding encoder;



    void Start()
    {
        latestRecievedMsg = "";
    }

    public void Connect(string ServerIP, string ServerPort)
    {
        IsConnected = false;
        encoder = new System.Text.ASCIIEncoding();
        socket = new StreamSocket();

        socket.Control.QualityOfService = SocketQualityOfService.LowLatency;

        HostName networkHost = new HostName(ServerIP.Trim());
        IAsyncAction outstandingAction = socket.ConnectAsync(networkHost, ServerPort);
        AsyncActionCompletedHandler aach = new AsyncActionCompletedHandler(NetworkConnectedHandler);
        outstandingAction.Completed = aach;
    }

    private void NetworkConnectedHandler(IAsyncAction asyncInfo, AsyncStatus status)
    {
        string cRes = "";
        if (status == AsyncStatus.Completed)
        {
            IsConnected = true;
            // cRes = "Succesfully connected to server.";
            // SendData("CONCT|" + userName);
            ListenerTask = new Task(DoListen);
            ListenerTask.Start();
            Debug.Log("Succesfully connected to server.");
        }
        else
        {
            IsConnected = false;
            socket.Dispose();
            socket = null;
            Debug.Log("Failed to connect to server.");
        }

        isConnecting = false;
    }


    private void DoListen()
    {
        // Debug.Log("moro");
        // Task.Delay(1500).Wait();
        // Debug.Log("tere");

        // Debug.Log("moro");
        // Debug.Log("tere");

        while (IsConnected)
        {
            // SendData("Hello from windows");
            // Debug.Log("Going to listen");
            SendData("antti_on_jeesus_jou_jou_jou");
            Listen();
            // Debug.Log("Outside listen");
        }
    }


    public async void SendData(string data)
    {
        while (IsSending)
            return;

        IsSending = true;
        bool sendStatus = false;
        DataWriter writer = new DataWriter(socket.OutputStream);
        byte[] bMsg = encoder.GetBytes(data);
        writer.WriteUInt32((uint)bMsg.Length);
        writer.WriteBytes(bMsg);
        // string msgType = data.Take(5).ToString();
        try
        {
            await writer.StoreAsync();
            await writer.FlushAsync();
            // Debug.Log("Succesfully send the message: " + data);
            sendStatus = true;
        }
        catch (Exception e)
        {
            switch (SocketError.GetStatus(e.HResult))
            {
                case SocketErrorStatus.Unknown:
                    // StatusUpdated(false, "Sending Failed: SocketErrorUnknow");
                    Debug.Log("Sending Failed: SocketErrorUnknow");
                    throw;
                case SocketErrorStatus.HostNotFound:
                    Debug.Log("Sending Failed: Host not found.");
                    break;
                case SocketErrorStatus.OperationAborted:
                    Debug.Log("Sending Failed: Execution Aborted.");
                    break;
                default:
                    Debug.Log("Sending Failed: Other Exception occured");
                    break;
            }
        }
        finally
        {
            writer.DetachStream();
            // Debug.Log("Succesfully send the message: " + data);
            IsSending = false;
            // StatusUpdated(sendStatus, res);
        }
    }





    private async void Listen()
    {
        if (!IsConnected || IsReading)
            return;

        IsReading = true;

        string strType = "";
        byte[] bData = null;
        DataReader reader = new DataReader(socket.InputStream);
        // reader.UnicodeEncoding = Windows.Storage.Streams.UnicodeEncoding.Utf8;
        // reader.ByteOrder = Windows.Storage.Streams.ByteOrder.LittleEndian;
        // reader.InputStreamOptions = InputStreamOptions.Partial;

        System.Text.ASCIIEncoding enc = new System.Text.ASCIIEncoding();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        string input = "";
        try
        {
            uint sizeFieldCount = await reader.LoadAsync(sizeof(uint));
            if (sizeFieldCount != sizeof(uint))
            {
                Debug.Log("Failed to read stream.");
                return;
            }

            uint dataLength = reader.ReadUInt32();
            uint actualDataLength = await reader.LoadAsync(dataLength);
            // Debug.Log("DataLength: " + dataLength.ToString());
            if (dataLength != actualDataLength)
            {
                Debug.Log("Failed to read stream.");
                return;
            }

            byte[] receivedData = new byte[actualDataLength];
            reader.ReadBytes(receivedData);
            // byte[] bType = receivedData.Take(5).ToArray();
            // bData = receivedData.Skip(5).Take((int)actualDataLength - 5).ToArray();
            strType = enc.GetString(receivedData);
            // Debug.Log("Successfully received message: " + strType);



        }
        catch (ObjectDisposedException)
        {
            Debug.Log("here ObjectDisposedException exeption.");
            Task.Delay(10).Wait();
        }
        catch (System.Runtime.InteropServices.COMException)
        {
            Debug.Log("Lost connection to server.");
            CloseConnection();
        }
        catch (Exception e)
        {
            switch (SocketError.GetStatus(e.HResult))
            {
                case SocketErrorStatus.Unknown:
                    Debug.Log("Exception from Listening: Error Type: " + e.GetType().ToString() + " Message: " + e.Message);
                    throw;
                case SocketErrorStatus.HostNotFound:
                    Debug.Log("Reading Failed. Host not found.");
                    break;
                case SocketErrorStatus.OperationAborted:
                    Debug.Log("Connection was terminated correctly.");
                    break;
                default:
                    Debug.Log("Reading failed. Other Exception.");
                    // LineReceived("DSCON", null);
                    break;
            }
        }
        finally
        {
            // log.Trace("StreamSocketConnection | Listen | Finished.");
            // Debug.Log("StreamSocketConnection Listen Finished.");
            reader.DetachStream();
            watch.Stop();
            var elapsedMs = watch.ElapsedMilliseconds;
            // Debug.Log("Elapsed: " + elapsedMs);
            // Debug.Log("Got " + input.Length + " bytes");
            // Debug.Log("###########");
            // Debug.Log("Msg: " + input);
            // Debug.Log("###########");
            latestRecievedMsg = strType;
            // Task.Delay(10).Wait();
            IsReading = false;
        }
    }

    public void Disconnect()
    {
        // SendData("DSCON|" + userName);
        CloseConnection();
    }

    ~SSListener()
    {
        Disconnect();
    }
    public void ServerClosed()
    {
        CloseConnection();
    }
    private async void CloseConnection()
    {
        if (socket != null)
        {
            IsReading = false;
            IsSending = false;
            IsConnected = false;
            ListenerTask.Wait(100);
            await socket.CancelIOAsync();
            socket.Dispose();
            socket = null;
            Debug.Log("Disconnected from server.");
        }
        else
        {
            IsReading = false;
            IsSending = false;
            IsConnected = false;
            socket = null;
            Debug.Log("No connection to server present.");

        }
    }




    // Update is called once per frame
    void Update () {
        if (IsConnected)
        {
            ButtonText.text = "Tap to Disconnect";
        }
        else
        {
            ButtonText.text = "Tap to Connect";
        }

    }

#endif

#if UNITY_EDITOR
    private void ConnectUnity(string host, string port)
    {
        try
        {
            if (exchangeThread != null) CloseSocket();
            client = new System.Net.Sockets.TcpClient(host, Int32.Parse(port));
            stream = client.GetStream();
            reader = new StreamReader(stream);
            writer = new StreamWriter(stream) { AutoFlush = true };

            RestartExchange();
            Debug.Log("StreamSocket Connected!");
            IsConnected = true;
        }
        catch (Exception e)
        {
            Debug.Log("Error connecting to StreamSocket: " + e.ToString());
        }
    }

    public void CloseSocket()
    {
        exchangeStopRequested = true;
        if (exchangeThread != null)
        {
            exchangeThread.Abort();
            stream.Close();
            client.Close();
            writer.Close();
            reader.Close();
            stream = null;
            exchangeThread = null;
        }

        IsConnected = false;
        writer = null;
        reader = null;
    }

    public void RestartExchange()
    {
        if (exchangeThread != null) CloseSocket();
        exchangeStopRequested = false;
        exchangeThread = new Thread(ExchangePackets);
        exchangeThread.Start();
    }

    // https://gist.github.com/danielbierwirth/0636650b005834204cb19ef5ae6ccedb
    public void ExchangePackets()
    {
        while (!exchangeStopRequested)
        {
            if (writer == null || reader == null) continue;
            string received = "";
            byte[] bytes = new byte[client.SendBufferSize];
            int recv = 0;
            while (true)
            {
                recv = stream.Read(bytes, 0, client.SendBufferSize);
                received += Encoding.UTF8.GetString(bytes, 0, recv);
                if (received.EndsWith("\n")) break;
                // if (received.Length == 4096) break;
            }

            Debug.Log("Number of bytes " + received.Length);
            Debug.Log("Got " + received);
            System.Threading.Thread.Sleep(1000);
            // if (received == "empty\n") continue;
            latestRecievedMsg = received;
        }
    }



#endif

    void OnTapped()
    {
#if !UNITY_EDITOR
        Debug.Log("OnSelect SStreamver_2s");
        if (!IsConnected)
        {
            Connect(ServerIP, ServerPort);
        }
        else
        {
            CloseConnection();
        }
#endif

#if UNITY_EDITOR
        Debug.Log("Connector OnSelect");
        if (!IsConnected)
        {
            Debug.Log("Connector tapped");
            ConnectUnity(ServerIP, ServerPort);
        }
        else
        {
            Debug.Log("Disconnecting");
            CloseSocket();
        }
#endif
    }

}
