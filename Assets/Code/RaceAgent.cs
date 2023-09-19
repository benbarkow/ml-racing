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
    //reward parameters
    public float maxVelAngle = 45.0f;
    public float maxSpeed = 0.18f;
    public float curveAngleDriftThreshold = 40.0f;
    public float targetDriftAngle = 45.0f;
    public List<float[]> curves = new List<float[]>();
    public float curveThreashold = 150.0f;
    private List<float> checkpointDistances = new List<float>();


    public float SteerSpeedDegPerSec = 100f;
    public PathCreator pathCreator;
    public Camera cam;
    public RacetrackGenerator racetrackGenerator;

    public float initZ = 0.0f;

    private float spawnPadding;

    void Start()
    {
        //cache all later requiered components
        VPcontrol = GetComponent<VehiclePhysics.VPVehicleController>();
        VPinput = GetComponent<VehiclePhysics.VPStandardInput>();
        VPbase = GetComponent<VehiclePhysics.VehicleBase>();
        rb = GetComponent<Rigidbody>();
        imu = GetComponentInChildren<IntertialMeasurementUnit>();
        distanceCoveredCanvas = GameObject.Find("DistanceCoveredCanvas").GetComponent<Canvas>();
        distanceCoveredCanvas.transform.rotation = cam.transform.rotation;
        rewardCanvasPrefab = GameObject.Find("RewardCanvas").GetComponent<Canvas>();
        rewardCanvasPrefab.transform.rotation = cam.transform.rotation;
        rewardCanvasList = new List<Tuple<int, Canvas>>();
        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);
        Random.InitState(System.DateTime.Now.Millisecond);
        trackPoints = new List<Vector3>();
        totalDistanceText = distanceCoveredCanvas.GetComponentInChildren<TMPro.TextMeshProUGUI>();
        spawnPadding = 10.0f;
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
        // remove all curve spheres
        foreach(GameObject sphere in curveSpheres){
            Destroy(sphere);
        }
        //remove reward canvas list
        foreach(Tuple<int, Canvas> rewardCanvasTuple in rewardCanvasList){
            Destroy(rewardCanvasTuple.Item2.gameObject);
        }
        rewardCanvasList = new List<Tuple<int, Canvas>>();

        // racetrackGenerator.generateRandomCircle();
        // racetrackGenerator.pickRandom();
        // racetrackGenerator.generateNew();
        racetrackGenerator.generateRandomTrack();
        //init curves
        // InitCurves();
        // InitCheckpoints();
    
        //reset to start position
        // This resets the vehicle and 'drops' it from a height of 0.5m (so that it does not clip into the ground and get stuck)
        // VehiclePhysics.VPResetVehicle.ResetVehicle(VPbase, initZ, false);

        //reset vehicle velocity
        // rb.velocity = 0.0f * this.transform.forward 
        //     + 0.0f * this.transform.up 
        //     + 0.0f * this.transform.right;
        // rb.angularVelocity = 0.0f * this.transform.forward 
        //     + 0.0f * this.transform.up 
        //     + 0.0f * this.transform.right;


        // startDistanceOnPath = getMostStraightDistOnPath();
        startDistanceOnPath = pathCreator.path.length - spawnPadding;
        // startDistanceOnPath = Random.Range(spawnPadding, pathCreator.path.length);
        //random selection from beginning and end of track
        // startDistanceOnPath = spawnPadding;
        // int direction = Random.Range(0, 2) * 2 - 1;
        // startDistanceOnPath = pathCreator.path.length - spawnPadding;

        //initPos 
        // int direction = Random.Range(0, 2) * 2 - 1;
        int direction = -1;
        ResetVehicleOnPath(startDistanceOnPath, direction);

        // this.transform.localPosition = startPosition;
        rb.isKinematic = false;
        imu.rBody.isKinematic = false;

        // //start velocity of 20
        // rb.velocity = 0.0f * this.transform.forward;
        // rb.velocity = this.transform.forward * 6.0f;

        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);
        prevDistanceOnPath = startDistanceOnPath;
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // sensor.AddObservation(imu.LocalVelocity.x / 300f);
        // sensor.AddObservation(imu.LocalVelocity.z / 300f);

        // sensor.AddObservation(-rb.angularVelocity.y / 10f);

        // sensor.AddObservation(Mathf.Abs(imu.SideSlip) / 180f);

        // sensor.AddObservation(VPcontrol.data.Get(Channel.Vehicle, VehicleData.EngineRpm) / (1000.0f * VPcontrol.engine.maxRpm ));
    }

    private void HandleHeuristics(ActionBuffers actionBuffers) {
        if (actionBuffers.ContinuousActions[0] == 0f){
            //VPinput.externalSteer = SmoothSteering(imu.SideSlip / VPcontrol.steering.maxSteerAngle); //Gyro
            VPinput.externalSteer = SmoothSteering(-rb.angularVelocity.y * 0.030516f);        //mapping to degrees per second);
        }
        else{
            VPinput.externalSteer = SmoothSteering(actionBuffers.ContinuousActions[0]);
        }
        VPinput.externalThrottle = PathMathSupports.Remap(actionBuffers.ContinuousActions[1], 0f, 1f, 0f, 1f);
    }


    

    private float getMostStraightDistOnPath()
    {
        float trackLength = pathCreator.path.length;
        float pointDistance = 0.5f;
        float measureDistance = 20.0f;

        int pointCount = Mathf.FloorToInt(trackLength / pointDistance);
        Vector3 pointFromIndex(int index)
        {
            if (index < 0)
            {
                index = pointCount + index;
            }
            if (index >= pointCount)
            {
                index = index - pointCount;
            }
            return pathCreator.path.GetPointAtDistance(index * pointDistance);
        }

        Tuple<float, float> maxStraight = new Tuple<float, float>(0.0f, 0.0f);

        for (int i = 0; i < pointCount; i++)
        {
            Vector3 currPoint = pointFromIndex(i);
            Vector3 nextPoint = pointFromIndex(i + Mathf.FloorToInt(measureDistance / pointDistance));
            Vector3 prevPoint = pointFromIndex(i - Mathf.FloorToInt(measureDistance / pointDistance));

            Vector3 currToNext = nextPoint - currPoint;
            Vector3 currToPrev = prevPoint - currPoint;
            float angle = Vector3.Angle(currToNext, currToPrev);
            if (angle > maxStraight.Item2)
            {
                maxStraight = new Tuple<float, float>(i * pointDistance, angle);
            }
        }
        return maxStraight.Item1;
    }

    private void InitCheckpoints(){
        checkpointDistances = new List<float>();
        float trackLength = pathCreator.path.length;
        float pointDistance = 5.0f;
        
        for(float i = 0.0f; i < trackLength; i += pointDistance){
            checkpointDistances.Add(i);
            //add sphere at checkpoint
            // Vector3 point = pathCreator.path.GetPointAtDistance(i);
            // GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            // //disable sphere collider
            // sphere.GetComponent<SphereCollider>().enabled = false;
            // sphere.transform.position = point;
            // sphere.GetComponent<MeshRenderer>().material.color = Color.red;
            // curveSpheres.Add(sphere);
        }
    }

    private void InitCurves(){
        curves = new List<float[]>();
        float trackLength = pathCreator.path.length;
        float pointDistance = 0.5f;
        float measureDistance = 8.0f;

        int pointCount = Mathf.FloorToInt(trackLength / pointDistance);
        Tuple<Vector3, float> getInfoFromIndex (int index)
        {
            if (index < 0)
            {
                index = pointCount + index;
            }
            if (index >= pointCount)
            {
                index = index - pointCount;
            }
            //pathCreator.path.GetPointAtDistance(index * pointDistance);
            return new Tuple<Vector3, float>(pathCreator.path.GetPointAtDistance(index * pointDistance), index*pointDistance);
        }

        bool isBetween(float value, float[] range){
            if(range[0] < range[1]){
                if(value > range[0] && value < range[1]){
                    return true;
                }
            }
            else if(range[0] > range[1]){
                if(value > range[0] || value < range[1]){
                    return true;
                }
            }
            return false;
        }

        Tuple<float, float> maxStraight = new Tuple<float, float>(0.0f, 0.0f);

        for (int i = 0; i < pointCount; i++)
        {

            Tuple<Vector3, float> currPointInfo = getInfoFromIndex(i);
            Tuple<Vector3, float> nextPointInfo = getInfoFromIndex(i + Mathf.FloorToInt(measureDistance / pointDistance));
            Tuple<Vector3, float> prevPointInfo = getInfoFromIndex(i - Mathf.FloorToInt(measureDistance / pointDistance));

            Vector3 currPoint = currPointInfo.Item1;
            Vector3 nextPoint = nextPointInfo.Item1;
            Vector3 prevPoint = prevPointInfo.Item1;

            Vector3 currToNext = nextPoint - currPoint;
            Vector3 currToPrev = prevPoint - currPoint;
            float angle = Vector3.Angle(currToNext, currToPrev);
            if (angle < 90)
            {
                maxStraight = new Tuple<float, float>(i * pointDistance, angle);
            }
            if(angle < curveThreashold){
                bool modifiedExistingCurve = false;
                for(int j = 0; j < curves.Count; j++){
                    if(isBetween(prevPointInfo.Item2, curves[j]) || isBetween(currPointInfo.Item2, curves[j])){
                        curves[j][1] = nextPointInfo.Item2;
                        modifiedExistingCurve = true;
                        break;
                    }
                }
                if(!modifiedExistingCurve){
                    float[] newCurve = new float[2];
                    newCurve[0] = prevPointInfo.Item2;
                    newCurve[1] = nextPointInfo.Item2;
                    curves.Add(newCurve);
                }
            }
        }
        // //combine overlapping curves and very close curves
        // float closeCurveThreashold = 3.0f;
        // for(int i = 0; i < curves.Count; i++){
        //     //check if end of curve is close to start of next curve

        // }

        //print out curve arrays
        // Debug.Log("------------------------------------------------------------");
        // for(int i = 0; i < curves.Count; i++){
        //     String line = "";
        //     foreach(float distance in curves[i]){
        //         line += distance.ToString() + ", ";
        //     }
        //     Debug.Log("curve " + i.ToString() + ": " + line);
        // }

        //place sphere at start point of track
        // GameObject startSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        // startSphere.GetComponent<SphereCollider>().enabled = false;
        // startSphere.transform.position = pathCreator.path.GetPointAtDistance(0.0f);
        // startSphere.GetComponent<MeshRenderer>().material.color = Color.red;
        // curveSpheres.Add(startSphere);

        // //place spheres on every curve with defferent color
        // foreach(float[] curve in curves){
        //     foreach(float distance in curve){
        //         Vector3 point = pathCreator.path.GetPointAtDistance(distance);
        //         GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //         //disable sphere collider
        //         sphere.GetComponent<SphereCollider>().enabled = false;
        //         sphere.transform.position = point;
        //         curveSpheres.Add(sphere);
        //         //different color for start and end of curve
        //         if(distance == curve[0]){
        //             sphere.GetComponent<MeshRenderer>().material.color = Color.green;
        //         }
        //         else{
        //             sphere.GetComponent<MeshRenderer>().material.color = Color.blue;
        //         }

        //     }
        // }

        //print out curve arrays
        // for(int i = 0; i < curves.Count; i++){
        //     String line = "";
        //     foreach(float distance in curves[i]){
        //         line += distance.ToString() + ", ";
        //     }
        //     Debug.Log("curve " + i.ToString() + ": " + line);
        // }
    }

    

    private float[] VelAngleReward(float distanceOnPath){
         //get direction of travel
        Vector3 direction = rb.velocity.normalized;
        Vector2 direction2D = new Vector2(direction.x, direction.z);
        // Debug.Log("direction: " + direction2D.ToString());
        //get angle between direction and path tangent
        Vector3 tangent = pathCreator.path.GetDirectionAtDistance(distanceOnPath) * carDirection;
        Vector2 tangent2D = new Vector2(tangent.x, tangent.z);
        //angle as value between 0 and 90
        float velocityValue = new Vector2(rb.velocity.x, rb.velocity.z).magnitude;
        float angle = Vector2.Angle(direction2D, tangent2D);
        if(velocityValue < 1.0f){
            angle = 0.0f;
        }
        // if(angle > 90.0f){
        //     angle = 180.0f - angle;
        // }

        // float b = -1/8;
        // float a = (1 - 4*b) / 16;
        // float velocityReward = Mathf.Min(a * Mathf.Pow(velocityValue, 2) + b * velocityValue, 1.0f);

        // //set negative reward proportional to angle and scale with velocity value
        // float angleReward = Mathf.Max(1-(angle / maxVelAngle), 0.0f);
        // angleReward = angleReward * velocityReward;
        // Debug.Log("angle reward: " + angleReward.ToString());
        if(angle > maxVelAngle && StepCount > 20){
            return new float[] {-1.0f, angle};
        }
        return new float[] {1.0f, angle};
    }

    private float CheckpointReward(){
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        float reward = 0.0f;
        for(int i = 0; i < checkpointDistances.Count; i++){
            if(distanceOnPath > prevDistanceOnPath){
                if(distanceOnPath > checkpointDistances[i] && prevDistanceOnPath < checkpointDistances[i]){
                    // Debug.Log("distance on path: " + distanceOnPath.ToString() + " prev distance on path: " + prevDistanceOnPath.ToString() + " checkpoint distance: " + checkpointDistances[i].ToString());
                    // checkpointDistances.RemoveAt(i);
                    reward = 1.0f;
                    break;
                }
            }
            else if(distanceOnPath < prevDistanceOnPath){
                if(distanceOnPath < checkpointDistances[i] && prevDistanceOnPath > checkpointDistances[i]){
                    // Debug.Log("distance on path: " + distanceOnPath.ToString() + " prev distance on path: " + prevDistanceOnPath.ToString() + " checkpoint distance: " + checkpointDistances[i].ToString());
                    // checkpointDistances.RemoveAt(i);
                    reward = 1.0f;
                    break;
                }
            }

        }
        prevDistanceOnPath = distanceOnPath;
        return reward;
    }

    private float SpeedReward(float distanceOnPath){

        float speed = (Mathf.Abs(distanceOnPath - prevDistanceOnPath));

        //if passed start line
        if(speed > 1.0f){
            speed = pathCreator.path.length - speed;
        }
        
        prevDistanceOnPath = distanceOnPath;
        //set reward and norm with maxStepDistance with max reward of 1
        float distReward = Mathf.Min(speed / maxSpeed, 1.0f);
        return distReward;
    }

    private float RunofPenalty(float distanceOnPath){
        //calculate distance to center of the road
        Vector3 closestPointOnPath = pathCreator.path.GetPointAtDistance(distanceOnPath);
        float distanceToCenter = Vector3.Distance(this.transform.position, closestPointOnPath);
        // // Debug.Log("distance to road center: " + distanceToCenter.ToString());
        // //if car runs off the road
        // float centerDistRewardNegative = (Mathf.Pow(distanceToCenter/3.0f, 2));
        // // Debug.Log("center distance reward: " + centerDistRewardNegative.ToString());
        if(distanceToCenter > 3.0 && StepCount > 20){
            //reset car position
            return -1.0f;
        }
        return 1.0f;
        // return centerDistRewardNegative;
    }

    private float GoalReward(float distanceOnPath, float velAngle){
        //calculate distance to center of the road
        if(distanceOnPath < 30.0f){
            return (5.0f * (1 - (velAngle / maxVelAngle)));
        }
        return -1.0f;
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        HandleHeuristics(actionBuffers);

        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);

        // if(Mathf.Abs(imu.SideSlip)-45f > 40f){
        //     SetReward(0.0f);
        //     EndEpisode();
        //     return;
        // }
       
        float speedReward = SpeedReward(distanceOnPath);

        float runofPenalty = RunofPenalty(distanceOnPath);
        if(runofPenalty == -1.0f){
            SetReward(-0.1f);
            EndEpisode();
            return;
        }
        // // Debug.Log("run of penalty: " + runofPenalty.ToString());

        float[] angleReward = VelAngleReward(distanceOnPath);
        if(angleReward[0] == -1.0f){
            SetReward(-0.1f);
            EndEpisode();
            return;
        }

        // float goalReward = GoalReward(distanceOnPath, angleReward[1]);
        // if(goalReward != -1.0f){
        //     // Debug.Log("goal reward: " + goalReward.ToString());
        //     // Debug.Log("step count: " + StepCount.ToString());
        //     SetReward(goalReward);
        //     EndEpisode();
        //     return;
        // }

        // float steerSpeedReward = SteerSpeedReward();

        // float checkpointReward = CheckpointReward();

        // if(speedReward < 0.5f){
        //     speedReward = 0.0f;
        // }

        //drift reward
        float driftReward = DriftReward(distanceOnPath);

        //calculate total reward
        // float reward = driftReward*(runofPenalty*((speedReward * 6 + 4*angleReward) / 10));
        // float reward = speedReward*(angleReward - runofPenalty);
        // float reward = (driftReward * 4 + speedReward) / 5;
        float reward = (speedReward + driftReward*9) / 10;
        // float reward = 1.0f;
        // float reward = (speedReward*7 + angleReward*3)/10;
        // float reward = speedReward;
        // float reward = (7*driftReward + 3*speedReward)/10;
        // float reward = checkpointReward;

        SetReward(reward);
        rewardEvent(reward, carDirection, reward == 1.0f);
        // rewardEvent(reward, carDirection);
        // Debug.Log("reward: " + GetCumulativeReward().ToString());
        cumulativeRewardEvent(GetCumulativeReward());
        // totalDistanceText.text = (driftReward).ToString();
    }

    private float SteerSpeedReward(){
        float maxSteerDiff = 0.16f;
        float currentSteeringAngle = VPinput.externalSteer;
        float steeringAngleDiff = Mathf.Abs(currentSteeringAngle - previousSteeringAgle);
        float steerSpeedReward = Mathf.Max(0.0f, 1.0f - (steeringAngleDiff / maxSteerDiff));
        previousSteeringAgle = VPinput.externalSteer;
        return steerSpeedReward;
    }

     private float DriftReward(float distanceOnPath) {

        float agentDistance = distanceOnPath;

        float driftScale = 1f;
        float curveExpandEntry = 0.0f;
        float curveExpandExit = -4.0f;
        float smoothingDistance = 4.0f;

        bool inCurve = false;

        //curve points
        Vector3[] curvePoints = new Vector3[3];

        //init curve Points one 10.0f before and one 10.0f after agent distance
        curvePoints[0] = pathCreator.path.GetPointAtDistance(agentDistance + 2.5f);
        curvePoints[1] = pathCreator.path.GetPointAtDistance(agentDistance - 2.5f);
        curvePoints[2] = pathCreator.path.GetPointAtDistance(agentDistance - 7.5f);

        // for(int i = 0; i < curves.Count; i++){
        //     if(
        //         carDirection == 1 &&
        //         agentDistance > curves[i][0] - curveExpandEntry &&
        //         agentDistance < curves[i][1] + curveExpandExit
        //     ){
        //         inCurve = true;
        //         curvePoints[0] = pathCreator.path.GetPointAtDistance(curves[i][0]);
        //         float inBetweenDist = curves[i][0] + (curves[i][1] - curves[i][0]) / 2;
        //         if(curves[i][0] > curves[i][1]){
        //             inBetweenDist = curves[i][0] + pathCreator.path.length - curves[i][1] + (curves[i][1] - curves[i][0]) / 2;
        //         }
        //         if(inBetweenDist > pathCreator.path.length){
        //             inBetweenDist = inBetweenDist - pathCreator.path.length;
        //         }
        //         curvePoints[1] = pathCreator.path.GetPointAtDistance(inBetweenDist);
        //         curvePoints[2] = pathCreator.path.GetPointAtDistance(curves[i][1]);
        //         //complute drift scale between 0 and 1 and increases with viewer distance to curve center in smooting distance
        //         if(agentDistance > curves[i][0] - curveExpandEntry && agentDistance < curves[i][0] - curveExpandEntry + smoothingDistance){
        //             driftScale = Mathf.Min(1.0f, (agentDistance - (curves[i][0] - curveExpandEntry)) / smoothingDistance);
        //         }
        //         else if(agentDistance < curves[i][1] + curveExpandExit && agentDistance > curves[i][1] + curveExpandExit - smoothingDistance){
        //             driftScale = Mathf.Min(1.0f, ((curves[i][1] + curveExpandExit) - agentDistance) / smoothingDistance);
        //         }
        //     }
        //     else if(
        //         carDirection == -1 &&
        //         agentDistance < curves[i][1] + curveExpandEntry &&
        //         agentDistance > curves[i][0] - curveExpandExit
        //     ){
        //         inCurve = true;
        //         curvePoints[0] = pathCreator.path.GetPointAtDistance(curves[i][1]);
        //         float inBetweenDist = curves[i][0] + (curves[i][1] - curves[i][0]) / 2;
        //         if(curves[i][0] > curves[i][1]){
        //             inBetweenDist = curves[i][0] + pathCreator.path.length - curves[i][1] + (curves[i][1] - curves[i][0]) / 2;
        //         }
        //         if(inBetweenDist > pathCreator.path.length){
        //             inBetweenDist = inBetweenDist - pathCreator.path.length;
        //         }
        //         curvePoints[1] = pathCreator.path.GetPointAtDistance(inBetweenDist);
        //         curvePoints[2] = pathCreator.path.GetPointAtDistance(curves[i][0]);
        //         //complute drift scale between 0 and 1 and increases with viewer distance to curve center in smooting distance
        //         if(agentDistance < curves[i][1] + curveExpandEntry && agentDistance > curves[i][1] + curveExpandEntry - smoothingDistance){
        //             driftScale = Mathf.Min(1.0f, ((curves[i][1] + curveExpandEntry) - agentDistance) / smoothingDistance);
        //         }
        //         else if(agentDistance > curves[i][0] - curveExpandExit && agentDistance < curves[i][0] - curveExpandExit + smoothingDistance){
        //             driftScale = Mathf.Min(1.0f, (agentDistance - (curves[i][0] - curveExpandExit)) / smoothingDistance);
        //         }
        //     }
        // }
        inCurve = true;
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

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        //steer
        continuousActionsOut[0] = 0f;
        if( Input.GetKey(KeyCode.D) ) continuousActionsOut[0] = 1f;
        if( Input.GetKey(KeyCode.A) ) continuousActionsOut[0] = -1f;
        
        //throttle
        continuousActionsOut[1] = 0.0f;
        if( Input.GetKey(KeyCode.W) ) continuousActionsOut[1] = 1f;
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