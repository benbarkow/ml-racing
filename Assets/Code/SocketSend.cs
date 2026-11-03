using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehiclePhysics;
using PathCreation;
using TMPro;
using System;
using Random=UnityEngine.Random;
using System.Net;
using System.Net.Sockets;
using System.Text;


public class SocketSend: MonoBehaviour
{
    public Camera carCam;

    private float lastCamUpdate;
    public float camUpdateInterval = 0.0f;

    //neworking
    private TcpListener listener;
    private TcpClient client;
    private NetworkStream nwStream;

    private VehiclePhysics.VPVehicleController VPcontrol;
    private VehiclePhysics.VPStandardInput VPinput;

    private List<byte[]> imageBuffer = new List<byte[]>();
    private bool isSent = true;

    private RenderTexture renderTexture;

	private int sendCounter  = 0;
	private float sendTime = 0.0f;
	public bool debug = false;

    private bool connected = false;

    public bool isConnected()
    {
        return connected;
    }

    public void disconnect()
    {
        if(!connected){
            return;
        }
        nwStream.Close();
        client.Close();
        listener.Stop();
        connected = false;
    }

	public bool connect()
    {
        if(connected){
            return true;
        }
        int tryPort = 8080;
        // Initialize socket
        listener = new TcpListener(IPAddress.Any, tryPort);
		if(debug){
			return true;
		}

        bool portFound = false;
        try 
        {
            listener = new TcpListener(IPAddress.Any, tryPort);
            listener.Start();
            portFound = true;
            connected = true;
        }
        catch (SocketException)
        {
            // Continue to the next iteration to try the next port
        }

        if(!portFound) 
        {
            Debug.LogError("No available ports in the specified list!");
            return false;  // Or handle this situation as appropriate for your application.
        }

        client = listener.AcceptTcpClient();
        nwStream = client.GetStream();
        return true;
    } 

    byte[] CaptureCameraToBytes(Camera cam)
    {
        if (renderTexture == null)
        {
            renderTexture = new RenderTexture(80, 60, 24);
        }

        cam.targetTexture = renderTexture;
        cam.Render();
        RenderTexture.active = renderTexture;

        Texture2D tex = new Texture2D(80, 60, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, 80, 60), 0, 0);
        tex.Apply();

        byte[] bytes = tex.EncodeToJPG(); // Use JPG instead of PNG for faster encoding

        Destroy(tex);
        cam.targetTexture = null;
        RenderTexture.active = null;

        return bytes;
    }

    void Update()
    {
        //clean up spheres
		if(Time.time - sendTime > 1f){
			// Debug.Log("Image Stacks Hz: " + sendCounter);
			sendTime = Time.time;
			sendCounter = 0;
		}
        byte[] cameraFrame = CaptureCameraToBytes(carCam);
        imageBuffer.Add(cameraFrame);
        if(imageBuffer.Count > 3)
        {
            imageBuffer.RemoveAt(0);
        }
    }

    public void sendPath(
        PathCreator pathCreator, 
        Vector2 initCarPosition, 
        Vector2 initCarPathPosition,
        Vector2 initCarDirection,
        Vector2 initCarPathDirection
    )
    {
        if(!connected){
            return;
        }
		//send data delimiter "<START>" 
		byte[] start = Encoding.ASCII.GetBytes("<START_PATH>");
		byte[] end = Encoding.ASCII.GetBytes("<END_PATH>");

		List<Vector3> trackPoints = new List<Vector3>();
        float pointDistanceOnPath = 0.1f;
        for(float i = 0; i < pathCreator.path.length; i += pointDistanceOnPath)
        {
            trackPoints.Add(pathCreator.path.GetPointAtDistance(i));
        }

		List<Vector2> trackPoints2D = new List<Vector2>();
		foreach(Vector3 point in trackPoints){
			trackPoints2D.Add(new Vector2(point.x, point.z));
		}

		//send number of points
		nwStream.Write(start, 0, start.Length);
		nwStream.Write(BitConverter.GetBytes(trackPoints2D.Count), 0, 4);
		//send points
		foreach(Vector2 point in trackPoints2D){
			nwStream.Write(BitConverter.GetBytes(point.x), 0, 4);
			nwStream.Write(BitConverter.GetBytes(point.y), 0, 4);
		}

        //send initial car position
        nwStream.Write(BitConverter.GetBytes(initCarPosition.x), 0, 4);
        nwStream.Write(BitConverter.GetBytes(initCarPosition.y), 0, 4);
        //send initial car path position
        nwStream.Write(BitConverter.GetBytes(initCarPathPosition.x), 0, 4);
        nwStream.Write(BitConverter.GetBytes(initCarPathPosition.y), 0, 4);
        //send initial car direction
        nwStream.Write(BitConverter.GetBytes(initCarDirection.x), 0, 4);
        nwStream.Write(BitConverter.GetBytes(initCarDirection.y), 0, 4);
        //send initial car path direction
        nwStream.Write(BitConverter.GetBytes(initCarPathDirection.x), 0, 4);
        nwStream.Write(BitConverter.GetBytes(initCarPathDirection.y), 0, 4);
        //send end

		nwStream.Write(end, 0, end.Length);
    }

    public void sendData(float[] targetFeatures, Vector2 carPosition, Vector2 carDirection, int[] actions){
        if(!connected){
            return;
        }

        //concatinate all the values into one array
        List<float> allValues = new List<float>();
        allValues.AddRange(targetFeatures);
        allValues.Add(carPosition.x);
        allValues.Add(carPosition.y);
        allValues.Add(carDirection.x);
        allValues.Add(carDirection.y);
        //add actions as floats
        allValues.AddRange(new List<float>(Array.ConvertAll(actions, x => (float)x)));
        float[] allValuesArray = allValues.ToArray();

        //target features to byte array
        byte[] featuresBytes = new byte[allValuesArray.Length * 4];
        Buffer.BlockCopy(allValuesArray, 0, featuresBytes, 0, featuresBytes.Length);

        //send data
        if(imageBuffer.Count == 3){
            sendDataOverSocket(imageBuffer, featuresBytes);
        }
    }

    public void sendEnd(int epSteps)
    {
        //send data delimiter "<START>" 
        byte[] end = Encoding.ASCII.GetBytes("<Abort>");

        nwStream.Write(BitConverter.GetBytes(end.Length), 0, 4);
        nwStream.Write(end, 0, end.Length);

        //send length of steps

        nwStream.Write(BitConverter.GetBytes(epSteps), 0, 4);
        //send images
    }

    private void sendDataOverSocket(List<byte[]> images, byte[] features)
    {


        //send data delimiter "<START>" 
        byte[] start = Encoding.ASCII.GetBytes("<START>");
        byte[] end = Encoding.ASCII.GetBytes("<END>");

        nwStream.Write(start, 0, start.Length);
        nwStream.Write(BitConverter.GetBytes(features.Length), 0, 4);
        nwStream.Write(features, 0, features.Length);
        //send images
        //send number of images
        nwStream.Write(BitConverter.GetBytes(images.Count), 0, 4);
        //first image lengths
        foreach(byte[] image in images)
        {
            nwStream.Write(BitConverter.GetBytes(image.Length), 0, 4);
            nwStream.Write(image, 0, image.Length);
        }
        nwStream.Write(end, 0, end.Length);
    }
}