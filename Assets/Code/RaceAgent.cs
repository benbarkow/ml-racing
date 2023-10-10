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
    private float startDistanceOnPath;
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
    private float SteerSpeedDegPerSec = 360f;
    //reward parameters


    public PathCreator[] paths;
    public int fixedPathIndex;
    public float[] fixedStartDistancesOnPaths; 
    public int[] fixedDirections;

    private PathCreator pathCreator;

  private float[] curvatureRange = new float[2]{-18.2f, 18.2f};
    private float[] speedRange = new float[2] { 0.0f, 16.0f };
    private float[] rotationAngleRange = new float[2] { -60.0f, 60.0f };
    private float[] positionRange = new float[2] { -3f, 3f };
    private float[] offsetRangeForReward = new float[2] { -3.2f, 3.2f };

    private float[] angularVelocityRange = new float[2] { -170.0f, 170.0f };
    private float[] minMaxVelX = new float[2]{0.0f, 16.0f};
    private float[] minMaxVelY = new float[2]{-5.5f, 5.5f};

    private int direction;

    private int envStepCount;

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
        envStepCount = 0;
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

    public override void OnEpisodeBegin()
    {

        //remove reward canvas list
        foreach(Tuple<int, Canvas> rewardCanvasTuple in rewardCanvasList){
            Destroy(rewardCanvasTuple.Item2.gameObject);
        }
        rewardCanvasList = new List<Tuple<int, Canvas>>();

        //choose random path
        pathCreator = paths[Random.Range(0, paths.Length)];
        if(fixedPathIndex != -1){
            pathCreator = paths[fixedPathIndex];
        }

        startDistanceOnPath = Random.Range(0, pathCreator.path.length);
        if(fixedPathIndex != -1){
            startDistanceOnPath = fixedStartDistancesOnPaths[fixedPathIndex];
        }

        //initPos 
        direction = Random.Range(0, 2) * 2 - 1;
        if(fixedPathIndex != -1){
            direction = fixedDirections[fixedPathIndex];
        }

        ResetVehicleOnPath(startDistanceOnPath, direction);

        // this.transform.localPosition = startPosition;
        rb.isKinematic = false;
        imu.rBody.isKinematic = false;

        VPcontrol.data.Set(Channel.Input, InputData.AutomaticGear, 1);
        prevDistanceOnPath = startDistanceOnPath;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //  //car velocity in car coordinates
        // Vector2 carVelocityGlobal = new Vector2(rb.velocity.x, rb.velocity.z);
        // //car velocity in this.transform coordinates
        // Vector2 carForward = new Vector2(transform.forward.x, transform.forward.z);
        // float transformAngle = Mathf.Atan2(carForward.y, carForward.x);

        // float[][] rotationMatrix = new float[][] {
        //     new float[] {Mathf.Cos(transformAngle), Mathf.Sin(transformAngle)},
        //     new float[] {-Mathf.Sin(transformAngle), Mathf.Cos(transformAngle)},
        // };

        // Vector2 carVelocity = new Vector2(
        //     rotationMatrix[0][0] * carVelocityGlobal.x + rotationMatrix[0][1] * carVelocityGlobal.y,
        //     rotationMatrix[1][0] * carVelocityGlobal.x + rotationMatrix[1][1] * carVelocityGlobal.y
        // );

        // float normalVel = PathMathSupports.Remap(Math.Abs(carVelocity.x), minMaxVelX[0], minMaxVelX[1], 0.0f, 1.0f );
        // float normalVelPerpendicular = PathMathSupports.Remap(carVelocity.y, minMaxVelY[0], minMaxVelY[1], 0f, 1.0f );

        // sensor.AddObservation(normalVel);
        // sensor.AddObservation(normalVelPerpendicular);
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        float[] features = computeFeaturesAll(distanceOnPath);
        foreach(float feature in features){
            sensor.AddObservation(feature);
        }
    }

    float[] computeFeaturesAll(float distance){

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

    private float VelocityRewardRegular(float carVelocity)
    {
        float maxBaseVelocity = speedRange[1];

        float velocityRew = carVelocity / maxBaseVelocity;

        float velocityReClamped = Mathf.Clamp(velocityRew, 0.0f, 1.0f);
        return velocityReClamped;
    }

    private float VelocityRewardLinear(float carVelocity)
    {
        float maxBaseVelocity = speedRange[1];
        int stepsUntilMaxVelocity = 100000;

        //compute max velocity as linear function of steps
        double stepFactor = (double)StepCount / stepsUntilMaxVelocity;
        double maxDouble = (maxBaseVelocity * 0.2) + ((maxBaseVelocity * 0.8f) * stepFactor);
        float maxVelocity = (float)maxDouble;

        float velocityRew = carVelocity / maxBaseVelocity;

        float velocityReClamped = Mathf.Clamp(velocityRew, 0.0f, (float)(0.2f + 0.8f * stepFactor));
        return velocityReClamped;
    }

    private float VelocityRewardDynamic(float carVelocity, float[] curvatures)
    {
        float[] absCurvatures = new float[curvatures.Length];
        for(int i = 0; i < curvatures.Length; i++)
        {
            absCurvatures[i] = Mathf.Abs(curvatures[i]);
        }

        float maxBaseVelocity = speedRange[1];
        
        // absolute sum of curvatures
        float curvatureSum = 0.0f;
        foreach(float curvature in absCurvatures)
        {
            curvatureSum += curvature;
        }

        float curvatureMax = Mathf.Max(absCurvatures);
        // Debug.Log(curvatureMax);
        float k = 2.0f;
        float maxVelocity = Mathf.Exp(-k * curvatureMax) * maxBaseVelocity;

        float velocityRew = carVelocity / maxVelocity;
        if (carVelocity > maxVelocity)
        {
            float velDiff = carVelocity - maxVelocity;
            velocityRew = Mathf.Clamp(1 - velDiff / maxVelocity, 0.0f, 1.0f);
        }
        if(velocityRew < -0.05f && StepCount > 20)
        {
            return -1.0f;
        }
        float velocityReClamped = Mathf.Clamp(velocityRew, 0.0f, 1.0f);
        return velocityReClamped;
    }

    private float OffsetReward(float offset){
        float centerRew = 1.0f - Mathf.Pow((Mathf.Abs(offset) / offsetRangeForReward[1]), 2.0f);
        float centerRewClamped = Mathf.Clamp(centerRew, 0.0f, 1.0f);
        return centerRewClamped;
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

        // if(!inCurve){
        //     return 1.0f;
        // }

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

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        HandleHeuristics(actionBuffers);
        envStepCount++;


        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
       
        float[] features = computeFeatures(distanceOnPath);

        float[] trackCurvatures = features[3..];

        float offsetReward = OffsetReward(features[0]);
        if(offsetReward == 0.0f){
            SetReward(0.0f);
            Debug.Log("Offset end");
            EndEpisode();
            return;
        }

        // float velocityReward = VelocityRewardDynamic(features[1], trackCurvatures);
        float velocityReward = VelocityRewardRegular(features[1]);
        if(velocityReward == -1.0f){
            SetReward(0.0f);
            Debug.Log("VelEnd");
            EndEpisode();
            return;
        }

        float reward = velocityReward * offsetReward;
       
        SetReward(reward);
        rewardEvent(reward, carDirection, reward == 1.0f);
      
        cumulativeRewardEvent(GetCumulativeReward());
    }

    float[] computeFeatures(float distance){

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
        Vector2 pathForward = new Vector2(pathTanget.x, pathTanget.z);
        float transformAnglePath = Mathf.Atan2(pathForward.y, pathForward.x);
        float transformAngle = Mathf.Atan2(carForward.y, carForward.x);


        float[][] rotationMatrix = new float[][] {
            new float[] {Mathf.Cos(transformAngle), Mathf.Sin(transformAngle)},
            new float[] {-Mathf.Sin(transformAngle), Mathf.Cos(transformAngle)},
        };

        float[][] rotationMatrixPath = new float[][] {
            new float[] {Mathf.Cos(transformAnglePath), Mathf.Sin(transformAnglePath)},
            new float[] {-Mathf.Sin(transformAnglePath), Mathf.Cos(transformAnglePath)},
        };

        Vector2 carVelocityPath = new Vector2(
            rotationMatrixPath[0][0] * carVelocityGlobal.x + rotationMatrixPath[0][1] * carVelocityGlobal.y,
            rotationMatrixPath[1][0] * carVelocityGlobal.x + rotationMatrixPath[1][1] * carVelocityGlobal.y
        );

        Vector2 carVelocity = new Vector2(
            rotationMatrix[0][0] * carVelocityGlobal.x + rotationMatrix[0][1] * carVelocityGlobal.y,
            rotationMatrix[1][0] * carVelocityGlobal.x + rotationMatrix[1][1] * carVelocityGlobal.y
        );

        float sideSlipAngleDeg = Mathf.Atan2(carVelocity.y, carVelocity.x) * Mathf.Rad2Deg;
        //relative velocity in car coordinates
        float[] trackCurvatures = getTrackCurvatures(distance);

		// Debug.Log(speedRelativeToPath.x);
		// Debug.Log(offset);
        //normalize features
        float[] normalCurvatures = new float[3] {
            PathMathSupports.Remap(trackCurvatures[0], curvatureRange[0], curvatureRange[1], -1.0f, 1.0f, true),
            PathMathSupports.Remap(trackCurvatures[1], curvatureRange[0], curvatureRange[1], -1.0f, 1.0f, true),
            PathMathSupports.Remap(trackCurvatures[2], curvatureRange[0], curvatureRange[1], -1.0f, 1.0f, true)
        };

        // float normalPosition = PathMathSupports.Remap(offset, positionRange[0], positionRange[1], 0.0f, 1.0f);
        // float normalRotation = PathMathSupports.Remap(theta, rotationAngleRange[0], rotationAngleRange[1], 0.0f, 1.0f);
        // float normalAngularVelocity = PathMathSupports.Remap(angularVelocity, angularVelocityRange[0], angularVelocityRange[1], 0.0f, 1.0f);
        // float normalVel = PathMathSupports.Remap(Math.Abs(carVelocity.x), minMaxVelX[0], minMaxVelX[1], 0.0f, 1.0f );
        // float normalVelPerpendicular = PathMathSupports.Remap(carVelocity.y, minMaxVelY[0], minMaxVelY[1], 0.0f, 1.0f );

        float[] features = new float[6] {
            offset,
            carVelocityPath.x,
            sideSlipAngleDeg,
            normalCurvatures[0],
            normalCurvatures[1],
            normalCurvatures[2]
        };
        return features;
    }

    float[] getTrackCurvatures(float currDistance)
    {
        //destroy old spheres
        // foreach(GameObject sphere in spheres)
        // {
        //     Destroy(sphere);
        // }
        float pointDistance = 2.5f;
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

    private void cumulativeRewardEvent(float reward){
        totalDistanceText.text = Math.Round(reward).ToString();
    }

    private void rewardEvent(float reward, int direction, bool checkpointPassed) {
        int maxStepsVisible = 100;
        for(int i = 0; i < rewardCanvasList.Count; i++){
            Tuple<int, Canvas> rewardCanvasTuple = rewardCanvasList[i];
            int stepsLeft = rewardCanvasTuple.Item1;
            Canvas canvas = rewardCanvasTuple.Item2;
            TMPro.TextMeshProUGUI text = canvas.GetComponentInChildren<TMPro.TextMeshProUGUI>();
            text.color = new Color(text.color.r, text.color.g, text.color.b, (float)stepsLeft / maxStepsVisible);
            if(stepsLeft == 1){
                Destroy(canvas.gameObject);
                rewardCanvasList.RemoveAt(i);
                i--;
            }
            else{
                rewardCanvasList[i] = Tuple.Create(stepsLeft-1, canvas);
            }
        }
        if(StepCount % 50 != 0 && reward != -1.0f){
            return;
        }
        //max of 2 digits after decimal point
        float backOffset = direction * 6.0f;
        Vector3 targetPosition = pathCreator.path.GetPointAtDistance(pathCreator.path.GetClosestDistanceAlongPath(this.transform.position)-backOffset);
        targetPosition.y = targetPosition.y + 0.5f;

        Canvas rewardCanvas = Instantiate(rewardCanvasPrefab);
        rewardCanvas.transform.position = targetPosition;
        TMPro.TextMeshProUGUI rewardText = rewardCanvas.GetComponentInChildren<TMPro.TextMeshProUGUI>();
        rewardText.text = Math.Round(reward, 2).ToString();
        if(reward > 0.0f){
            rewardText.color = Color.green;
        }
        else{
            rewardText.color = Color.red;
        }
        rewardCanvasList.Add(new Tuple<int, Canvas>(maxStepsVisible, rewardCanvas));
    }

    private void HandleHeuristics(ActionBuffers actionBuffers) {
        // if (actionBuffers.ContinuousActions[0] == 0f){
        //     //VPinput.externalSteer = SmoothSteering(imu.SideSlip / VPcontrol.steering.maxSteerAngle); //Gyro
        //     VPinput.externalSteer = SmoothSteering(-rb.angularVelocity.y * 0.030516f);        //mapping to degrees per second);
        // }
        // else{
        //     VPinput.externalSteer = SmoothSteering(actionBuffers.ContinuousActions[0]);
        // }
        // if(actionBuffers.ContinuousActions[1] == 0.0f){
        //     VPinput.externalThrottle = 0.0f;
        //     VPinput.externalBrake = 0.0f;
        // }
        // else{
        //     if(actionBuffers.ContinuousActions[1] > 0.0f){
        //         VPinput.externalThrottle = PathMathSupports.Remap(actionBuffers.ContinuousActions[1], 0f, 1f, 0f, 1f);
        //         VPinput.externalBrake = 0.0f;
        //     }
        //     else{
        //         VPinput.externalThrottle = 0.0f;
        //         Debug.Log(actionBuffers.ContinuousActions[1]);
        //         VPinput.externalBrake = PathMathSupports.Remap(actionBuffers.ContinuousActions[1], 0, -1f, 0f, 1f);
        //     }
        // }

        //same but in discrete actions

        if (actionBuffers.DiscreteActions[0] == 5){
            //VPinput.externalSteer = SmoothSteering(imu.SideSlip / VPcontrol.steering.maxSteerAngle); //Gyro
            VPinput.externalSteer = SmoothSteering(-rb.angularVelocity.y * 0.030516f);        //mapping to degrees per second);
        }
        else{
            VPinput.externalSteer = SmoothSteering(PathMathSupports.Remap(actionBuffers.DiscreteActions[0], 0, 10, -1, 1));
        }
        if(actionBuffers.DiscreteActions[1] == 5){
            VPinput.externalThrottle = 0.0f;
            VPinput.externalBrake = 0.0f;
        }
        else{
            VPinput.externalBrake = PathMathSupports.Remap(actionBuffers.DiscreteActions[1], 0, 5, 1, 0, true);
            VPinput.externalThrottle = PathMathSupports.Remap(actionBuffers.DiscreteActions[1], 5, 10, 0, 1, true);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // var continuousActionsOut = actionsOut.ContinuousActions;

        // //steer
        // continuousActionsOut[0] = 0f;
        // if( Input.GetKey(KeyCode.D) ) continuousActionsOut[0] = 1f;
        // if( Input.GetKey(KeyCode.A) ) continuousActionsOut[0] = -1f;
        
        // //throttle
        // continuousActionsOut[1] = 0.0f;
        // if( Input.GetKey(KeyCode.W) ) continuousActionsOut[1] = 1f;
        // if( Input.GetKey(KeyCode.S) ) continuousActionsOut[1] = -1f;

        //same but with discrete actions [0-10, 0-10]
        var discreteActionsOut = actionsOut.DiscreteActions;

        //steer
        discreteActionsOut[0] = 5;
        if( Input.GetKey(KeyCode.A) ) discreteActionsOut[0] = 0;
        if( Input.GetKey(KeyCode.D) ) discreteActionsOut[0] = 10;

        //throttle
        discreteActionsOut[1] = 5;
        if( Input.GetKey(KeyCode.S) ) discreteActionsOut[1] = 0;
        if( Input.GetKey(KeyCode.W) ) discreteActionsOut[1] = 10;

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