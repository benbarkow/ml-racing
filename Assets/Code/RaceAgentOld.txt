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
    private float prevDistanceCovered;
    private float lastRewardVisualizedAtStep;
    private Canvas rewardCanvasPrefab;
    private Canvas distanceCoveredCanvas;
    private List<Tuple<int, Canvas>> rewardCanvasList = new List<Tuple<int, Canvas>>();
    private float PreviousSteerDirection;
    private List<GameObject> curveSpheres = new List<GameObject>();

    private int carDirection = 1;
    //reward parameters
    public float maxVelAngle = 60.0f;
    public float maxSpeed = 0.2f;
    public float curveAngleDriftThreshold = 40.0f;
    public float targetDriftAngle = 45.0f;


    public float SteerSpeedDegPerSec = 100f;
    public PathCreator pathCreator;
    public Camera cam;
    public RacetrackGenerator racetrackGenerator;

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
        prevDistanceCovered = 0.0f;
        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);
        Random.InitState(System.DateTime.Now.Millisecond);

    }

    public override void OnEpisodeBegin()
    {
        racetrackGenerator.generateNew();
        //reset to start position
        // This resets the vehicle and 'drops' it from a height of 0.5m (so that it does not clip into the ground and get stuck)
        VehiclePhysics.VPResetVehicle.ResetVehicle(VPbase, 0, true);

        float maxPathLength = pathCreator.path.length;
        //compute a random distance along the path
        startDistanceOnPath = Random.Range(0.0f, maxPathLength);

        //initPos 
        Vector3 initPos = pathCreator.path.GetPointAtDistance(startDistanceOnPath);
        this.transform.position = new Vector3(initPos.x, initPos.y + 0.2f, initPos.z);
        //increase y position to avoid clipping into the ground
        Quaternion initRotation = pathCreator.path.GetRotationAtDistance(startDistanceOnPath);
        this.transform.rotation = initRotation;
        this.transform.Rotate(0.0f, 0.0f, 90.0f, Space.Self);
        //random direction 0 or 1
        int direction = Random.Range(0, 2);
        if (direction == 0){
            this.transform.Rotate(0.0f, 180.0f, 0.0f, Space.Self);
            carDirection = -1;
        }else {
            carDirection = 1;
        }

        startTransform = this.transform;

        // this.transform.localPosition = startPosition;
        rb.isKinematic = false;
        imu.rBody.isKinematic = false;

        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //add imu velocity as observation
        Vector3 localVel = imu.LocalVelocity;
        Vector2 localVelXZ = new Vector2(localVel.x, localVel.z);
        //get magnitude of local velocity
        float localVelMag = localVelXZ.magnitude;
        float velValue = Mathf.Min(localVelMag / 15f, 1f);
        sensor.AddObservation(velValue);

        //steering angle as observation between 0 and 1
        float steeringValue = (CurrentSteerDirection + 1f) / 2f;
        sensor.AddObservation(steeringValue);
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

    

    private float VelAngleReward(){
         //get direction of travel
        Vector3 direction = rb.velocity.normalized;
        Vector2 direction2D = new Vector2(direction.x, direction.z);
        // Debug.Log("direction: " + direction2D.ToString());
        //get angle between direction and path tangent
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        Vector3 tangent = pathCreator.path.GetDirectionAtDistance(distanceOnPath);
        Vector2 tangent2D = new Vector2(tangent.x, tangent.z);
        //angle as value between 0 and 90
        float velocityValue = new Vector2(rb.velocity.x, rb.velocity.z).magnitude;
        float angle = Vector2.Angle(direction2D, tangent2D);
        if(velocityValue < 1.0f){
            angle = 0.0f;
        }
        if(angle > 90.0f){
            angle = 180.0f - angle;
        }

        float b = -1/8;
        float a = (1 - 4*b) / 16;
        float velocityReward = Mathf.Min(a * Mathf.Pow(velocityValue, 2) + b * velocityValue, 1.0f);

        //set negative reward proportional to angle and scale with velocity value
        float angleReward = Mathf.Max(1-(angle / maxVelAngle), 0.0f);
        angleReward = angleReward * velocityReward;
        // Debug.Log("angle reward: " + angleReward.ToString());
        if(angle > maxVelAngle && StepCount > 20){
            rewardEvent(-1.0f, carDirection);
            EndEpisode();
            return -1.0f;
        }
        return angleReward;
    }

    private float SpeedReward(){
        float distanceCovered = 0.0f;
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        if(distanceOnPath < startDistanceOnPath){
            // Debug.Log("distance covered is negative: "a + distanceOnPath.ToString() + " < " + startDistanceOnPath.ToString());
            distanceCovered = pathCreator.path.length - startDistanceOnPath + distanceOnPath;
        }
        else{
            distanceCovered = distanceOnPath - startDistanceOnPath;
        }
        float speed = (Mathf.Abs(distanceCovered - prevDistanceCovered));


        prevDistanceCovered = distanceCovered;
        //set reward and norm with maxStepDistance with max reward of 1
        float distReward = Mathf.Min(speed / maxSpeed, 1.0f);
        return distReward;
    }

    private float RunofPenalty(){
        //calculate distance to center of the road
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        Vector3 closestPointOnPath = pathCreator.path.GetPointAtDistance(distanceOnPath);
        float distanceToCenter = Vector3.Distance(this.transform.position, closestPointOnPath);
        // Debug.Log("distance to road center: " + distanceToCenter.ToString());
        //if car runs off the road
        float centerDistRewardNegative = Mathf.Max((1-Mathf.Pow(distanceToCenter/3, 8))-1, -1.0f);
        // Debug.Log("center distance reward: " + centerDistRewardNegative.ToString());
        if(distanceToCenter > 3.0 && StepCount > 20){
            rewardEvent(-1.0f, carDirection);
            SetReward(-1.0f);
            EndEpisode();
            return -1.0f;
        }
        return centerDistRewardNegative;
    }

    private float DriftReward() {
        //check if the agent is on a curve

        float trackLength = pathCreator.path.length;
        int trackPointsCount = pathCreator.path.NumPoints;
        float targetPointDistance = 5f;
        int targetPointCount = Mathf.FloorToInt(trackLength / targetPointDistance);

        //choose points on the track to get target point count
        List<Vector3> trackPoints = new List<Vector3>();

        for(int i = 0; i< targetPointCount; i++){
            float distance = i * targetPointDistance;
            Vector3 point = pathCreator.path.GetPointAtDistance(distance);
            trackPoints.Add(point);
        }

        //get 10 trackPoints ahead of the agent
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        int currentPointIndex = Mathf.FloorToInt(distanceOnPath / targetPointDistance) - 1*carDirection;
        if(currentPointIndex < 0){
            currentPointIndex = trackPoints.Count + currentPointIndex;
        }
        // foreach(GameObject curveSphere in curveSpheres){
        //     Destroy(curveSphere);
        // }
        curveSpheres = new List<GameObject>();
        Vector3[] curvePoints = new Vector3[5];
        for(int i = 0; i < 5; i ++){
            int index = currentPointIndex + i*carDirection;
            if(index >= trackPoints.Count){
                index = index - trackPoints.Count;
            }
            else if(index < 0){
                index = trackPoints.Count + index;
            }
            curvePoints[i] = trackPoints[index];
            // GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            // //disable sphere collider
            // sphere.GetComponent<SphereCollider>().enabled = false;
            // sphere.transform.position = trackPoints[index];
            // curveSpheres.Add(sphere);
        }
        //get the commulative angle between the 5 points
        float angle = 0.0f;
        for(int i = 0; i < 3; i++){
            Vector3 v1 = curvePoints[i+1] - curvePoints[i];
            Vector3 v2 = curvePoints[i+2] - curvePoints[i+1];
            angle += Vector3.Angle(v1, v2);
        }
        // Debug.Log("Angle: " + angle);

        if(angle < curveAngleDriftThreshold){
            return 1.0f;
        }
        Vector3 closestPathTangent = pathCreator.path.GetDirectionAtDistance(distanceOnPath);

        //get curve direction from the curve points (-1 for right, 1 for left)
        int curveAngleSign = -(int)Math.Sign(Vector3.Cross(curvePoints[2] - curvePoints[0], curvePoints[4] - curvePoints[2]).y);
        float carAngleSign = (int)Math.Sign(imu.SideSlip);

        //convert to -1 or 1
        if(carAngleSign != curveAngleSign){
            return -0.01f;
        }
        float rew = (1f / (1f + Mathf.Pow(Mathf.Abs((Mathf.Abs(imu.SideSlip) - 45f) / 25f),(2f*4f))));
        // float desiredAngleSign 
        return rew;
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        //calculate steer delta 
        // float maxSteerDelta = 0.044444f;
        // float steerDelta = Mathf.Min(Mathf.Abs(CurrentSteerDirection - PreviousSteerDirection)/maxSteerDelta, 1.0f);
        // float steerDeltaPenalty = Mathf.Pow(steerDelta, 4) * -0.3f;

        HandleHeuristics(actionBuffers);
       
        float speedReward = SpeedReward();

        float runofPenalty = RunofPenalty();
        if(runofPenalty == -1.0f){
            return;
        }

        float angleReward = VelAngleReward();
        if(angleReward == -1.0f){
            return;
        }

        float driftReward = DriftReward();

        //calculate total reward
        float reward = driftReward*((speedReward * 7 + angleReward) / 8) + runofPenalty;
        SetReward(reward);

        // rewardEvent(reward, carDirection);
        rewardEvent(reward, carDirection);
        cumulativeRewardEvent(GetCumulativeReward());
    }

    private void cumulativeRewardEvent(float distanceCovered){
        TMPro.TextMeshProUGUI totalDistanceText = distanceCoveredCanvas.GetComponentInChildren<TMPro.TextMeshProUGUI>();
        totalDistanceText.text = Math.Round(distanceCovered).ToString();

    }

    private void rewardEvent(float reward, int direction) {
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
        continuousActionsOut[1] = 0;
        if( Input.GetKey(KeyCode.W) ) continuousActionsOut[1] = 1f;
        if( Input.GetKey(KeyCode.S) ) continuousActionsOut[1] = 0f;
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