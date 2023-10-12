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


public class RaceAgent : Agent
{
    private Rigidbody rb;    
    private VehiclePhysics.VPStandardInput VPinput;
    private VehiclePhysics.VPVehicleController VPcontrol;
    private VehiclePhysics.VehicleBase VPbase;
    private IntertialMeasurementUnit imu;
    private Transform startTransform;
    private float CurrentSteerDirection;
    private float prevDistanceOnPath;
    private float lastRewardVisualizedAtStep;
    private Canvas rewardCanvasPrefab;
    private Canvas distanceCoveredCanvas;
    TMPro.TextMeshProUGUI totalDistanceText;
    private List<Tuple<int, Canvas>> rewardCanvasList = new List<Tuple<int, Canvas>>();
    private float previousSteeringAgle;
    private List<GameObject> curveSpheres = new List<GameObject>();
    private List<Vector3> trackPoints;
    private int carDirection = 1;
    private RenderTexture renderTexture;

    public Camera cam;
    public float SteerSpeedDegPerSec = 100f;
    public PathCreator pathCreator;
    public float initZ = 0.0f;
    public int direction = 1;
    public float startDistanceOnPath;
    public bool isTesting;

    private float spawnPadding;

    private List<byte[]> imageBuffer = new List<byte[]>();

    private float[] curvatureRange = new float[2]{-13.2f, 13.2f};
    private float[] speedRange = new float[2] { 0.0f, 16.0f };
    private float[] rotationAngleRange = new float[2] { -60.0f, 60.0f };
    private float[] positionRange = new float[2] { -2.95f, 2.95f };

    private float[] angularVelocityRange = new float[2] { -170.0f, 170.0f };
    private float[] minMaxVelX = new float[2]{0.0f, 16.0f};
    private float[] minMaxVelY = new float[2]{-5.5f, 5.5f};

    private SocketSend socketSend;

    private int[] latestAction = new int[2]{0, 0};
    private bool sentPath = false;

    void Start()
    {
        //cache all later requiered components
        VPcontrol = GetComponent<VehiclePhysics.VPVehicleController>();
        VPinput = GetComponent<VehiclePhysics.VPStandardInput>();
        VPbase = GetComponent<VehiclePhysics.VehicleBase>();
        rb = GetComponent<Rigidbody>();
        imu = GetComponentInChildren<IntertialMeasurementUnit>();
        distanceCoveredCanvas = GameObject.Find("DistanceCoveredCanvas").GetComponent<Canvas>();
        rewardCanvasPrefab = GameObject.Find("RewardCanvas").GetComponent<Canvas>();
        rewardCanvasList = new List<Tuple<int, Canvas>>();
        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);
        Random.InitState(System.DateTime.Now.Millisecond);
        trackPoints = new List<Vector3>();
        totalDistanceText = distanceCoveredCanvas.GetComponentInChildren<TMPro.TextMeshProUGUI>();
        spawnPadding = 10.0f;

        renderTexture = new RenderTexture(80, 60, 24);
        socketSend = GetComponent<SocketSend>();
        while(!socketSend.isConnected()){
            socketSend.connect();
        }
    }    
    public override void OnEpisodeBegin()
    {
        // this.transform.localPosition = startPosition;
        rb.isKinematic = false;
        imu.rBody.isKinematic = false;
        VPcontrol.data.Set(Channel.Input, InputData.AutomaticGear, 1);

        // int direction = Random.Range(0, 2) * 2 - 1;
        // startDistanceOnPath = Random.Range(0.0f, pathCreator.path.length);
        //reset car position
        ResetVehicleOnPath(startDistanceOnPath, direction);


    }   

    private void ResetVehicleOnPath(float distanceOnPath, int direction){
        Quaternion initRotation = pathCreator.path.GetRotationAtDistance(distanceOnPath);
        Vector3 initPosition = pathCreator.path.GetPointAtDistance(distanceOnPath);
        //set x and z of transform to init position
        this.transform.position = new Vector3(initPosition.x, this.transform.position.y, initPosition.z);
        this.transform.rotation = initRotation;
        this.transform.Rotate(0.0f, 0.0f, 90.0f, Space.Self);

        rb.velocity = 0.0f * this.transform.forward
            + 0.0f * this.transform.up
            + 0.0f * this.transform.right;

        rb.angularVelocity = 0.0f * this.transform.forward
            + 0.0f * this.transform.up
            + 0.0f * this.transform.right;

        //random direction -1 or 1
        if (direction == -1){
            this.transform.Rotate(0.0f, 180.0f, 0.0f, Space.Self);
            carDirection = -1;
        }else {
            carDirection = 1;
        }
        rb.velocity = 0.0f * this.transform.forward;
        VPinput.externalSteer = 0.0f;
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
            float nextDistance = currDistance + i * pointDistance * direction;
            if(nextDistance > pathCreator.path.length)
            {
                nextDistance = nextDistance - pathCreator.path.length;
            }
            pathPoints[i] = pathCreator.path.GetPointAtDistance(nextDistance);
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

    float[] computeFeatures(){
        float distance = pathCreator.path.GetClosestDistanceAlongPath(transform.position);

        float angularVelocityRad = rb.angularVelocity.y;
        float angularVelocity = rb.angularVelocity.y * Mathf.Rad2Deg;

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

		float offset = (p.x - u.x) * t.y - (p.y - u.y) * t.x;

		float magnitude(Vector2 v) {
			return ((float)Math.Sqrt(v.x * v.x + v.y * v.y));
		}

		float dot(Vector2 v1, Vector2 v2) {
			return v1.x * v2.x + v1.y * v2.y;
		}
		//get rotation angle from path and always use smallest angle so if angle is 150 it should be 30
		Vector3 forward = transform.forward;
		Vector3 tangent = pathTanget;

        float dotProduct = dot(new Vector2(forward.x, forward.z), new Vector2(tangent.x, tangent.z));
        float forwardMagnitude = magnitude(new Vector2(forward.x, forward.z));
        float tangentMagnitude = magnitude(new Vector2(tangent.x, tangent.z));
		    // Calculate the cosine of the angle
        float cosineTheta = dotProduct / (forwardMagnitude * tangentMagnitude);

        // Clamp the value between -1 and 1 to ensure stability
        cosineTheta = Mathf.Clamp(cosineTheta, -1f, 1f);

        // Calculate the angle in radians
        float thetaRadians = Mathf.Acos(cosineTheta);

        // Determine the sign of the angle using the cross product's z-component
        Vector3 crossProduct = Vector3.Cross(forward, tangent);
        if (crossProduct.z > 0)
        {
            thetaRadians = -thetaRadians;
        }

        // Convert the angle to degrees
        float theta = thetaRadians * (180f / Mathf.PI);

        Vector3 pathNormal = pathCreator.path.GetNormalAtDistance(distance);
        Vector2 pathNormal2D = new Vector2(pathNormal.x, pathNormal.z);
        //the speed in the direction of the path and the speed perpendicular to the path
        Vector2 speedRelativeToPath = new Vector2(Vector2.Dot(rb.velocity, pathTangent2D), Vector2.Dot(rb.velocity, pathNormal2D));
        
        //car velocity in car coordinates
        Vector2 carVelocityGlobal = new Vector2(rb.velocity.x, rb.velocity.z);
        //car velocity in this.transform coordinates
        Vector2 carForward = new Vector2(transform.forward.x, transform.forward.z);
        float transformAngle = Mathf.Atan2(carForward.y, carForward.x);

        float[][] rotationMatrix = new float[][] {
            new float[] {Mathf.Cos(transformAngle), Mathf.Sin(transformAngle)},
            new float[] {-Mathf.Sin(transformAngle), Mathf.Cos(transformAngle)},
        };

        Vector2 carVelocity = new Vector2(
            rotationMatrix[0][0] * carVelocityGlobal.x + rotationMatrix[0][1] * carVelocityGlobal.y,
            rotationMatrix[1][0] * carVelocityGlobal.x + rotationMatrix[1][1] * carVelocityGlobal.y
        );
        //relative velocity in car coordinates

        float[] trackCurvatures = getTrackCurvatures();

		// Debug.Log(speedRelativeToPath.x);
		// Debug.Log(offset);
        //normalize features
        float[] normalCurvatures = new float[3] {
            PathMathSupports.Remap(trackCurvatures[0], curvatureRange[0], curvatureRange[1], 0.0f, 1.0f),
            PathMathSupports.Remap(trackCurvatures[1], curvatureRange[0], curvatureRange[1], 0.0f, 1.0f),
            PathMathSupports.Remap(trackCurvatures[2], curvatureRange[0], curvatureRange[1], 0.0f, 1.0f)
        };
        float normalPosition = PathMathSupports.Remap(offset, positionRange[0], positionRange[1], 0.0f, 1.0f);
        float normalRotation = PathMathSupports.Remap(theta, rotationAngleRange[0], rotationAngleRange[1], 0.0f, 1.0f);
        float normalAngularVelocity = PathMathSupports.Remap(angularVelocity, angularVelocityRange[0], angularVelocityRange[1], 0.0f, 1.0f);
        float normalVel = PathMathSupports.Remap(Math.Abs(carVelocity.x), minMaxVelX[0], minMaxVelX[1], 0.0f, 1.0f );
        float normalVelPerpendicular = PathMathSupports.Remap(carVelocity.y, minMaxVelY[0], minMaxVelY[1], 0.0f, 1.0f );

        float[] features = new float[8] {
            normalCurvatures[0],
            normalCurvatures[1],
            normalCurvatures[2],
            normalPosition,
            normalRotation,
            normalAngularVelocity,
            normalVel,
            normalVelPerpendicular
        };
        return features;
    }

    private float SpeedReward(float distanceOnPath){

        // float speed = (Mathf.Abs(distanceOnPath - prevDistanceOnPath));

        // //if passed start line
        // if(speed > 1.0f){
        //     speed = pathCreator.path.length - speed;
        // }
        
        // prevDistanceOnPath = distanceOnPath;
        // //set reward and norm with maxStepDistance with max reward of 1
        // float distReward = Mathf.Min(speed / maxSpeed, 1.0f);
        float distReward = 1.0f;
        return distReward;
    }
    
    private float DriftReward(float distanceOnPath) {

        float agentDistance = distanceOnPath;

        float driftScale = 1f;

        //curve points
        Vector3[] curvePoints = new Vector3[3];

        //init curve Points one 10.0f before and one 10.0f after agent distance
        curvePoints[0] = pathCreator.path.GetPointAtDistance(agentDistance + 2.5f);
        curvePoints[1] = pathCreator.path.GetPointAtDistance(agentDistance - 2.5f);
        curvePoints[2] = pathCreator.path.GetPointAtDistance(agentDistance - 7.5f);

        driftScale = 1f;

        //calculate angle of curve
        float curveAngle = Vector3.Angle(curvePoints[1] - curvePoints[0], curvePoints[2] - curvePoints[1]);
        if(curveAngle > 90.0f){
            curveAngle = 180.0f - curveAngle;
        }
        if(curveAngle < 20.0f){
            return 0.0f;
        }

        // //get curve direction from the curve points (-1 for right, 1 for left)
        int curveAngleSign = -(int)Math.Sign(Vector3.Cross(curvePoints[1] - curvePoints[0], curvePoints[2] - curvePoints[1]).y);
        float carAngleSign = (int)Math.Sign(imu.SideSlip);

        // //convert to -1 or 1
        if(carAngleSign != curveAngleSign){
            return 0.0f;
        }
        float rew = Math.Min(1f, (((110f)*(1-driftScale) + 1) / (1f + Mathf.Pow(Mathf.Abs((Mathf.Abs(imu.SideSlip)-45f) / 25f),(2f*4f)))));

        // Factor in the driftScale
        rew *= driftScale;

        return rew;
    }
    
    private float RunofPenalty(){
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(transform.position);
        //calculate distance to center of the road
        Vector3 closestPointOnPath = pathCreator.path.GetPointAtDistance(distanceOnPath);
        float distanceToCenter = Vector3.Distance(this.transform.position, closestPointOnPath);
        // // Debug.Log("distance to road center: " + distanceToCenter.ToString());
        // //if car runs off the road
        // float centerDistRewardNegative = (Mathf.Pow(distanceToCenter/3.0f, 2));
        // // Debug.Log("center distance reward: " + centerDistRewardNegative.ToString());
        if(distanceToCenter > 3.3 && StepCount > 20){
            //reset car position
            return -1.0f;
        }
        return 1.0f;
        // return centerDistRewardNegative;
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        Vector2 carPosition2D = new Vector2(transform.position.x, transform.position.z);
        Vector2 carDirection2D = new Vector2(transform.forward.x, transform.forward.z);
        if(!sentPath && socketSend.isConnected()){
            Vector3 startPosition = pathCreator.path.GetPointAtDistance(startDistanceOnPath);
            Vector2 startPosition2D = new Vector2(startPosition.x, startPosition.z);
            Vector3 startDirection = pathCreator.path.GetDirectionAtDistance(startDistanceOnPath);
            Vector2 startDirection2D = new Vector2(startDirection.x, startDirection.z);

            socketSend.sendPath(pathCreator, carPosition2D, startPosition2D, carDirection2D, startDirection2D);
            sentPath = true;
        }
        float[] features = computeFeatures();
        socketSend.sendData(features, carPosition2D, carDirection2D, latestAction);
        foreach(float feature in features){
            sensor.AddObservation(feature);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        latestAction = new int[2]{actionBuffers.DiscreteActions[0], actionBuffers.DiscreteActions[1]};
        HandleHeuristics(actionBuffers);
        if(RunofPenalty() == -1.0f){
            sentPath = false;
            socketSend.sendEnd(StepCount);
            //sleep for 4 seconds
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        //steer
        discreteActionsOut[0] = 5;
        if( Input.GetKey(KeyCode.D) ) discreteActionsOut[0] = 10;
        if( Input.GetKey(KeyCode.A) ) discreteActionsOut[0] = 0;
        
        //throttle
        discreteActionsOut[1] = 5;
        if( Input.GetKey(KeyCode.W) ) discreteActionsOut[1] = 10;
        if( Input.GetKey(KeyCode.S) ) discreteActionsOut[1] = 0;
    } 
    
    private void HandleHeuristics(ActionBuffers actionBuffers) {
        if (actionBuffers.DiscreteActions[0] == 5){
            //VPinput.externalSteer = SmoothSteering(imu.SideSlip / VPcontrol.steering.maxSteerAngle); //Gyro
            VPinput.externalSteer = SmoothSteering(-rb.angularVelocity.y * 0.030516f);        //mapping to degrees per second);
        }
        else{
            VPinput.externalSteer = SmoothSteering(PathMathSupports.Remap(actionBuffers.DiscreteActions[0], 0, 10, -1, 1));
        }
        if(actionBuffers.DiscreteActions[1] > 5){
            VPinput.externalThrottle = PathMathSupports.Remap(actionBuffers.DiscreteActions[1], 5, 10, 0, 1);
            VPinput.externalBrake = 0.0f;
        }
        else{
            VPinput.externalBrake = PathMathSupports.Remap(actionBuffers.DiscreteActions[1], 0, 5, 1, 0);
            VPinput.externalThrottle = 0.0f; 
        }
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