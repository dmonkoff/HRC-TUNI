using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading;
using UnityEngine;
using Scripts;
using UnityEngine.UI;
using System.Text.RegularExpressions;

#if !UNITY_EDITOR
using System.Threading.Tasks;  
#endif





// https://foxypanda.me/tcp-client-in-a-uwp-unity-app-on-hololens/
public class TCP_connector : Singleton<TCP_connector>
{

#if !UNITY_EDITOR
    private bool _useUWP = true;
    private Windows.Networking.Sockets.StreamSocket socket;
    private Task exchangeTask;
#endif

#if UNITY_EDITOR
    System.Net.Sockets.TcpClient client;
    System.Net.Sockets.NetworkStream stream;
    private Thread exchangeThread;
#endif





    public static int n_bytes = 128;
    private Byte[] bytes = new Byte[n_bytes];
    private StreamWriter writer;
    private StreamReader reader;
    public string host = "192.168.10.106";
    public string port = "9001";
    public Text ButtonText = null;
    public string latestRecievedMsg;
    private bool connected = false;
    // private bool prev_state = false;
    private bool exchangeStopRequested = false;
    public string request_msg;


    private void Awake()
    {
        request_msg = "x\n";
    }

    
    private void Update()
    {

        if (connected)
        {
            ButtonText.text = "Tap to Disconnect";
        }
        else
        {
            ButtonText.text = "Tap to Connect";
        }


        /*
        if (connected != prev_state) {
            // BroadcastMessage("ConnectionState", connected);
            prev_state = connected;
        }

#if UNITY_EDITOR
        //Connecting in Unity play mode
        if (Input.GetKeyDown(KeyCode.I))
        {
            Debug.Log("StreamSocket Unity connecting");
            ConnectUnity();
        }

        //Disconnecting in Unity play mode
        if (Input.GetKeyDown(KeyCode.B))
        {
            Debug.Log("Disconnecting");
            CloseSocket();
        }
#endif

        if (connected) {
            ButtonText.text = "Tap to Disconnect";
        } else {
            ButtonText.text = "Tap to Connect";
        }
        */
    }

#if UNITY_EDITOR
    private void ConnectUnity()
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
            connected = true;
        }
        catch (Exception e)
        {
            Debug.Log("ERROR CONNECTING TO StreamSocket: " + e.ToString());
        }

    }
#endif

#if !UNITY_EDITOR
    private async void ConnectUWP() {
        try
        {
            if (exchangeTask != null) CloseSocket();

            socket = new Windows.Networking.Sockets.StreamSocket();
            Windows.Networking.HostName serverHost = new Windows.Networking.HostName(host);
            await socket.ConnectAsync(serverHost, port);

            Stream streamOut = socket.OutputStream.AsStreamForWrite();
            writer = new StreamWriter(streamOut) { AutoFlush = true };

            Stream streamIn = socket.InputStream.AsStreamForRead();
            reader = new StreamReader(streamIn);

            RestartExchange();
            connected = true;
        }
        catch (Exception e)
        {
            Debug.Log("ERROR WHILE TRYING TO CONNECT: "+e.ToString());
        }

}
#endif


    public void RestartExchange()
    {

#if UNITY_EDITOR
        if (exchangeThread != null) CloseSocket();
        exchangeStopRequested = false;
        exchangeThread = new System.Threading.Thread(ExchangePackets);
        exchangeThread.Start();
#endif



#if !UNITY_EDITOR
        if (exchangeTask != null) CloseSocket();
        exchangeStopRequested = false;
        exchangeTask = Task.Run(() => ExchangePackets());
#endif
    }



    //Tap Gesture on HL

    void OnTapped()
    {
#if !UNITY_EDITOR
        Debug.Log("Connector OnSelect");
        if ((InteractibleManager.Instance.FocusedGameObject == GameObject.Find("Connect_Button")) && (!connected))
        {
            Debug.Log("UWP Connector tapped");
            ConnectUWP();
        } 
        else if ((InteractibleManager.Instance.FocusedGameObject == GameObject.Find("Connect_Button")) && (connected))
        {
            Debug.Log("UWP Disconnecting");
            CloseSocket();
        }
#endif


#if UNITY_EDITOR
        Debug.Log("Connector OnSelect");
        if ((InteractibleManager.Instance.FocusedGameObject == GameObject.Find("Connect_Button")) && (!connected))
        {
            Debug.Log("Connector tapped");
            ConnectUnity();
        }
        else if ((InteractibleManager.Instance.FocusedGameObject == GameObject.Find("Connect_Button")) && (connected))
        {
            Debug.Log("Disconnecting");
            CloseSocket();
        }
#endif
    }


    public void ExchangePackets()
    {
        // TODO: maybe sleep here somewhere, the loop rolls really fast
        while (!exchangeStopRequested)
        {
            if (writer == null || reader == null) continue;

            // writer.Write(request_msg);
            string received = null;

#if UNITY_EDITOR
            byte[] bytes = new byte[client.SendBufferSize];
            int recv = 0;
            while (true)
            {
                recv = stream.Read(bytes, 0, client.SendBufferSize);
                received += Encoding.UTF8.GetString(bytes, 0, recv);
                if (received.EndsWith("\n")) break;
                // if (received.Length == 4096) break;
            }
#endif

#if !UNITY_EDITOR
            received = reader.ReadLine();
#endif

            // received = Regex.Replace(received, @"\t|\n|\r", ""); // remove the ending \n nonon this takes super much time
            Debug.Log("Msg length " + received.Length);
            // Debug.Log("Got message: " + received);
            if (received == "ok\n")
            {
                request_msg = "x\n";
            }

            if (received == "empty\n") continue;
            latestRecievedMsg = received;
        }
    }

    public void CloseSocket()
    {
        exchangeStopRequested = true;
#if UNITY_EDITOR
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
#endif
#if !UNITY_EDITOR
        Debug.Log("OnCloseSocket");
        if (exchangeTask != null) {
            exchangeTask.Wait();
            writer.Dispose();
            reader.Dispose();
            socket.Dispose();
            socket = null;
            exchangeTask = null;            
        }
#endif
        connected = false;
        writer = null;
        reader = null;
    }

    public void OnDestroy()
    {
        CloseSocket();
    }

}
