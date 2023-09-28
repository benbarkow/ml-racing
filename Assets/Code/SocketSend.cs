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


public class SocketSend : MonoBehaviour
{
    private Rigidbody rb;    
    //reward parameters
    public PathCreator pathCreator;
    public Camera carCam;
    public RacetrackGenerator racetrackGenerator;
    private VehiclePhysics.VPVehicleController VPcontrol;
    private float CurrentSteerDirection;
    public float SteerSpeedDegPerSec = 100f;
    private VehiclePhysics.VPStandardInput VPinput;

    private float radius;
    private float lastCamUpdate;
    public float camUpdateInterval = 100f;
    private float lastTrackUpdate;
    public float trackUpdateInterval = 10f;

    public float[] positionRange = new float[2] { -2.6f, 2.6f };
    public float[] speedRange = new float[2] { 0.0f, 10.0f };
    public float[] angularVelocityRange = new float[2] { -5.0f, 5.0f };

    public float[] rotationAngleRange = new float[2] { -60.0f, 60.0f };
    public float spawnDistancePadding = 0.0f;


    //neworking
    private TcpListener listener;
    private TcpClient client;
    private NetworkStream nwStream;

    private List<byte[]> imageBuffer = new List<byte[]>();
    private bool isSent = true;
    private List<int> portsToTry = new List<int> {
        8080,
    };  // Your list of ports to try

    private int direction = 1;
    private float offset = 0.0f;
    private float rotationAngle = 0.0f;
    private float[] minCurvatures = new float[3]{0.0f, 0.0f, 0.0f};
    private float[] maxCurvatures = new float[3]{0.0f, 0.0f, 0.0f};
    private float[] curvatureRange = new float[2]{-13.2f, 13.2f};
    private float[] minMaxRelPathVel = new float[2]{0.0f, 20.0f};
    private float[] minMaxPerpPathVel = new float[2]{-16.0f, 16.0f};
    private List<GameObject> spheres = new List<GameObject>();

    private RenderTexture renderTexture;

	private int sendCounter  = 0;
	private float sendTime = 0.0f;
	public bool debug = false;

    void Start()
    {
		sendTime = Time.time;
		lastCamUpdate = Time.time;
        VPcontrol = GetComponent<VehiclePhysics.VPVehicleController>();
        VPinput = GetComponent<VehiclePhysics.VPStandardInput>();
		rb = GetComponent<Rigidbody>();

        rb.isKinematic = false;
        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);

        renderTexture = new RenderTexture(80, 60, 24);
        rb = GetComponent<Rigidbody>();
        lastCamUpdate = Time.time;
        lastTrackUpdate = 0;

        // Initialize socket
        listener = new TcpListener(IPAddress.Any, portsToTry[0]);
		if(debug){
			return;
		}

        bool portFound = false;
        foreach(int tryPort in portsToTry) 
        {
            try 
            {
                listener = new TcpListener(IPAddress.Any, tryPort);
                listener.Start();
                portFound = true;
                break;  // Exit the loop as soon as a port is found
            }
            catch (SocketException)
            {
                // Continue to the next iteration to try the next port
            }
        }

        if(!portFound) 
        {
            Debug.LogError("No available ports in the specified list!");
            return;  // Or handle this situation as appropriate for your application.
        }

        client = listener.AcceptTcpClient();
        nwStream = client.GetStream();


		//send path info over socket
		sendTrackInfoOverSocket();
    } 
	

    void Update()
    {
		HandleHeuristics();	
        //clean up spheres
		if(Time.time - sendTime > 1f){
			Debug.Log("Image Stacks Hz: " + sendCounter);
			sendTime = Time.time;
			sendCounter = 0;
		}
        if(Time.time - lastCamUpdate < camUpdateInterval)
        {
            return;
        }
        lastCamUpdate = Time.time;
        // if(Time.time - lastTrackUpdate > trackUpdateInterval || lastTrackUpdate == 0)
        // {
        //     lastTrackUpdate = Time.time;
        //     // racetrackGenerator.generateRandomCircle(radius);
        // }
        //save image to buffer
        if(imageBuffer.Count < 3)
        {
            byte[] cameraFrame = CaptureCameraToBytes(carCam);
            imageBuffer.Add(cameraFrame);
            return;
        }
		direction = -1;

        float distance = pathCreator.path.GetClosestDistanceAlongPath(transform.position);

        float angularVelocity = rb.angularVelocity.y;

        Vector3 pathTanget = pathCreator.path.GetDirectionAtDistance(distance);
        if(direction == -1)
        {
            pathTanget = -pathTanget;
        }
        Vector2 pathTangent2D = new Vector2(pathTanget.x, pathTanget.z);

		//get offset from path
		Vector3 p_3 = pathCreator.path.GetPointAtDistance(distance);
		Vector2 u = new Vector2(p_3.x, p_3.z);
		Vector2 t = new Vector2(pathTanget.x, pathTanget.z).normalized;
		Vector2 p = new Vector2(transform.position.x, transform.position.z);

		offset = (p.x - u.x) * t.y - (p.y - u.y) * t.x;

		float magnitude(Vector2 v) {
			return ((float)Math.Sqrt(v.x * v.x + v.y * v.y));
		}

		float dot(Vector2 v1, Vector2 v2) {
			return v1.x * v2.x + v1.y * v2.y;
		}

		//get rotation angle from path and always use smallest angle so if angle is 150 it should be 30
		Vector3 forward = transform.forward;
		Vector3 tangent = pathTanget;
		float theta = Mathf.Acos(dot(forward, tangent) / (magnitude(forward) * magnitude(tangent)));

        Vector3 pathNormal = pathCreator.path.GetNormalAtDistance(distance);
        Vector2 pathNormal2D = new Vector2(pathNormal.x, pathNormal.z);
        //the speed in the direction of the path and the speed perpendicular to the path
        Vector2 speedRelativeToPath = new Vector2(Vector2.Dot(rb.velocity, pathTangent2D), Vector2.Dot(rb.velocity, pathNormal2D));

        float[] trackCurvatures = getTrackCurvatures();

		Debug.Log("-------------------");
		// Debug.Log(speedRelativeToPath.x);
		// Debug.Log(offset);
		Debug.Log(theta);
        //normalize features
        float[] normalCurvatures = new float[3] {
            PathMathSupports.Remap(trackCurvatures[0], curvatureRange[0], curvatureRange[1], 0.0f, 1.0f),
            PathMathSupports.Remap(trackCurvatures[1], curvatureRange[0], curvatureRange[1], 0.0f, 1.0f),
            PathMathSupports.Remap(trackCurvatures[2], curvatureRange[0], curvatureRange[1], 0.0f, 1.0f)
        };
        float normalPosition = PathMathSupports.Remap(offset, positionRange[0], positionRange[1], 0.0f, 1.0f);
        float normalRotation = PathMathSupports.Remap(theta, rotationAngleRange[0], rotationAngleRange[1], 0.0f, 1.0f);
        float normalAngularVelocity = PathMathSupports.Remap(angularVelocity, angularVelocityRange[0], angularVelocityRange[1], 0.0f, 1.0f);
        float normalVel = PathMathSupports.Remap(Math.Abs(speedRelativeToPath.x), minMaxRelPathVel[0], minMaxRelPathVel[1], 0.0f, 1.0f );
        float normalVelPerpendicular = PathMathSupports.Remap(speedRelativeToPath.y, minMaxPerpPathVel[0], minMaxPerpPathVel[1], 0.0f, 1.0f );

        float[] features = new float[10] {
            normalCurvatures[0],
            normalCurvatures[1],
            normalCurvatures[2],
            normalPosition,
            normalRotation,
            normalAngularVelocity,
            normalVel,
            normalVelPerpendicular,
			//add position x and z
			transform.position.x,
			transform.position.z
        };
        byte[] featuresBytes = new byte[features.Length * 4];
        Buffer.BlockCopy(features, 0, featuresBytes, 0, featuresBytes.Length);

		sendCounter++;
		if(!debug){
			sendDataOverSocket(imageBuffer, featuresBytes);
		}
        isSent = true;
        imageBuffer.Clear();
    }

    float[] getTrackCurvatures()
    {
        //destroy old spheres
        // foreach(GameObject sphere in spheres)
        // {
        //     Destroy(sphere);
        // }
        float pointDistance = 2.5f;
        float currDistance = pathCreator.path.GetClosestDistanceAlongPath(transform.position);
        Vector3[] pathPoints = new Vector3[9];
        for(int i = 0; i < 9; i++)
        {
            pathPoints[i] = pathCreator.path.GetPointAtDistance(currDistance + i * pointDistance * direction);
            //sphere for debugging
            // GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            // sphere.transform.position = pathPoints[i];
            // spheres.Add(sphere);
        }

        float[] curvatures = new float[3];

        for (int i = 0; i < 3; i++)
        {
            // Extract 3 points for each segment
            Vector3 point1 = pathPoints[i * 3];
            Vector3 point2 = pathPoints[i * 3 + 1];
            Vector3 point3 = pathPoints[i * 3 + 2];

            // Approximate the first and second derivatives using finite differences
            Vector3 deltaY = point3 - point1;
            Vector3 delta2Y = point3 - 2 * point2 + point1;

            // Calculate curvature
            float curvature = delta2Y.magnitude / Mathf.Pow(1 + deltaY.magnitude * deltaY.magnitude, 1.5f);

            // Determine direction of the curve (left or right) using cross product
            Vector3 crossProduct = Vector3.Cross(deltaY.normalized, delta2Y.normalized);
            if (crossProduct.y < 0) // Assuming Z is the up direction
            {
                curvature = -curvature; // Curve is to the left
            }

            // Scale the curvature for better visualization (adjust the scaling factor as needed)
            curvature *= 1000;

            curvatures[i] = curvature;
        }

        return curvatures;
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

	private void sendTrackInfoOverSocket(){
		//send data delimiter "<START>" 
		byte[] start = Encoding.ASCII.GetBytes("<START_PATH>");
		byte[] end = Encoding.ASCII.GetBytes("<END_PATH>");

		Vector3[] trackPoints = pathCreator.path.localPoints;
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
		nwStream.Write(end, 0, end.Length);

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
	private void HandleHeuristics() {
		if(Input.GetKey(KeyCode.D)) VPinput.externalSteer = SmoothSteering(1f);
		else if(Input.GetKey(KeyCode.A)) VPinput.externalSteer = SmoothSteering(-1f);
		else VPinput.externalSteer = SmoothSteering(0f);
		if(Input.GetKey(KeyCode.W)) VPinput.externalThrottle = 1f;
		else VPinput.externalThrottle = 0f;
    }

	private float SmoothSteering(float steerInput) {
        steerInput *= VPcontrol.steering.maxSteerAngle;
		float steer = CurrentSteerDirection * VPcontrol.steering.maxSteerAngle;
        float steerStepsPerSec = (1 / Time.fixedDeltaTime);
        float steerDegPerStep = (SteerSpeedDegPerSec / steerStepsPerSec);
        float steerError = Mathf.Abs(steer - steerInput);
        
        
        //stay within the allowed range. P control?
        if(steer < steerInput && steer + steerDegPerStep < steerInput) steer += steerDegPerStep;
        else if(steer < steerInput && steer + steerDegPerStep > steerInput) steer = steerInput;
        
        else if(steer > steerInput && steer - steerDegPerStep > steerInput) steer -= steerDegPerStep;
        else if(steer > steerInput && steer - steerDegPerStep < steerInput) steer = steerInput;


        CurrentSteerDirection = steer / VPcontrol.steering.maxSteerAngle;
		return CurrentSteerDirection;
	}
}