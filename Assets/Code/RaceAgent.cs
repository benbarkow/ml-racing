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
    private float PreviousSteerDirection;
    private List<GameObject> curveSpheres = new List<GameObject>();
    private List<Vector3> trackPoints;
    private int carDirection = 1;
    //reward parameters
    public float maxVelAngle = 60.0f;
    public float maxSpeed = 0.2f;
    public float curveAngleDriftThreshold = 40.0f;
    public float targetDriftAngle = 45.0f;
    public List<float[]> curves = new List<float[]>();


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
        rewardCanvasList = new List<Tuple<int, Canvas>>();
        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);
        Random.InitState(System.DateTime.Now.Millisecond);
        trackPoints = new List<Vector3>();
        totalDistanceText = distanceCoveredCanvas.GetComponentInChildren<TMPro.TextMeshProUGUI>();
    }

    public override void OnEpisodeBegin()
    {
        racetrackGenerator.generateNew();

        //init curves
        // InitCurves();
    
        //reset to start position
        // This resets the vehicle and 'drops' it from a height of 0.5m (so that it does not clip into the ground and get stuck)
        VehiclePhysics.VPResetVehicle.ResetVehicle(VPbase, 0, true);

        startDistanceOnPath = getMostStraightDistOnPath();
        prevDistanceOnPath = startDistanceOnPath;

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
    }

    private void HandleHeuristics(ActionBuffers actionBuffers) {
        if (actionBuffers.ContinuousActions[0] == 0f){
            //VPinput.externalSteer = SmoothSteering(imu.SideSlip / VPcontrol.steering.maxSteerAngle); //Gyro
            VPinput.externalSteer = SmoothSteering(-rb.angularVelocity.y * 0.030516f);        //mapping to degrees per second);
        }
        else{
            VPinput.externalSteer = SmoothSteering(actionBuffers.ContinuousActions[0]);
        }
        if(actionBuffers.ContinuousActions[1] >= 0f){
            VPinput.externalThrottle = PathMathSupports.Remap(actionBuffers.ContinuousActions[1], 0f, 1f, 0f, 1f);
            VPinput.externalBrake = 0f;
        }
        else{
            // VPinput.externalBrake = PathMathSupports.Remap(actionBuffers.ContinuousActions[1], 0f, -1f, 0f, 1f);
            // VPinput.externalThrottle = 0f;
        }
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

    private void InitCurves(){
        float trackLength = pathCreator.path.length;
        float pointDistance = 0.5f;

        // List<float[]> distanceRangesCurve = new List<float[]>();

        // for(int i = 0; i < trackLength/pointDistance; i++){
        //     Vector3 point = pathCreator.path.GetPointAtDistance(i*pointDistance);
        // }

        //remove curve spheres from previous episode
        for(int i = 0; i < curveSpheres.Count; i++){
            Destroy(curveSpheres[i]);
        }
        curveSpheres = new List<GameObject>();

        int pointCount = Mathf.FloorToInt(trackLength / pointDistance);
        float threshAngle = 2.5f;
        curves = new List<float[]>();
        List<float> tempCurve = new List<float>();
        for(int i = 0; i < pointCount; i++){
            Vector3 point = pathCreator.path.GetPointAtDistance(i*pointDistance);
            //disable sphere collider
            int prev_index = i-1;
            if(prev_index < 0){
                prev_index = pointCount - 1;
            }
            int next_index = i+1;
            if(next_index >= pointCount){
                next_index = 0;
            }
            Vector3 self_tangent = pathCreator.path.GetDirectionAtDistance(i*pointDistance);
            Vector3 prev_tangent = pathCreator.path.GetDirectionAtDistance(prev_index*pointDistance);
            Vector3 next_tangent = pathCreator.path.GetDirectionAtDistance(next_index*pointDistance);
            //angle between prev and next tangent
            float angle = Vector3.Angle(prev_tangent, next_tangent);
            if(angle > threshAngle){
                tempCurve.Add(i*pointDistance);
            }
            else{
                if(tempCurve.Count > 20){
                    curves.Add(tempCurve.ToArray());
                    tempCurve = new List<float>();
                }
                tempCurve = new List<float>();
            }
            angle = 0.0f;
        }

        //place spheres on every curve
        // foreach(float[] curve in curves){
        //     foreach(float distance in curve){
        //         Vector3 point = pathCreator.path.GetPointAtDistance(distance);
        //         GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //         //disable sphere collider
        //         sphere.GetComponent<SphereCollider>().enabled = false;
        //         sphere.transform.position = point;
        //         sphere.GetComponent<MeshRenderer>().material.color = Color.green;
        //         curveSpheres.Add(sphere);
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
            return -1.0f;
        }
        return angleReward;
    }

    private float SpeedReward(){
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);

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

    private float RunofPenalty(){
        //calculate distance to center of the road
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        Vector3 closestPointOnPath = pathCreator.path.GetPointAtDistance(distanceOnPath);
        float distanceToCenter = Vector3.Distance(this.transform.position, closestPointOnPath);
        // Debug.Log("distance to road center: " + distanceToCenter.ToString());
        //if car runs off the road
        float centerDistRewardNegative = (1-Mathf.Pow(distanceToCenter/3, 4));
        // Debug.Log("center distance reward: " + centerDistRewardNegative.ToString());
        if(distanceToCenter > 3.0 && StepCount > 20){
            return -1.0f;
        }
        return centerDistRewardNegative;
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        HandleHeuristics(actionBuffers);
       
        float speedReward = SpeedReward();

        float runofPenalty = RunofPenalty();
        if(runofPenalty == -1.0f){
            SetReward(-1.0f);
            EndEpisode();
            return;
        }

        float angleReward = VelAngleReward();
        if(angleReward == -1.0f){
            SetReward(-1.0f);
            EndEpisode();
            return;
        }

        //drift reward
        // float driftReward = DriftReward();

        //calculate total reward
        float reward = 1*(runofPenalty*((speedReward * 6 + 4*angleReward) / 10));

        SetReward(reward);

        // rewardEvent(reward, carDirection);
        rewardEvent(reward, carDirection);
        cumulativeRewardEvent(GetCumulativeReward());
    }

     private float DriftReward() {

        float agentDistance = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);

        float driftScale = 1f;
        float curveExpandEntry = 4.0f;
        float curveExpandExit = -2.0f;
        float smoothingDistance = 4.0f;

        bool inCurve = false;

        //curve points
        Vector3[] curvePoints = new Vector3[3];

        for(int i = 0; i < curves.Count; i++){
            if(
                carDirection == 1 &&
                agentDistance > curves[i][0] - curveExpandEntry &&
                agentDistance < curves[i][curves[i].Length-1] + curveExpandExit
            ){
                inCurve = true;
                curvePoints[0] = pathCreator.path.GetPointAtDistance(curves[i][0]);
                curvePoints[1] = pathCreator.path.GetPointAtDistance(curves[i][Mathf.FloorToInt(curves[i].Length/2)]);
                curvePoints[2] = pathCreator.path.GetPointAtDistance(curves[i][curves[i].Length-1]);
                //complute drift scale between 0 and 1 and increases with viewer distance to curve center in smooting distance
                if(agentDistance > curves[i][0] - curveExpandEntry && agentDistance < curves[i][0] - curveExpandEntry + smoothingDistance){
                    driftScale = Mathf.Min(1.0f, (agentDistance - (curves[i][0] - curveExpandEntry)) / smoothingDistance);
                }
                else if(agentDistance < curves[i][curves[i].Length-1] + curveExpandExit && agentDistance > curves[i][curves[i].Length-1] + curveExpandExit - smoothingDistance){
                    driftScale = Mathf.Min(1.0f, ((curves[i][curves[i].Length-1] + curveExpandExit) - agentDistance) / smoothingDistance);
                }
            }
            else if(
                carDirection == -1 &&
                agentDistance < curves[i][curves[i].Length-1] + curveExpandEntry &&
                agentDistance > curves[i][0] - curveExpandExit
            ){
                inCurve = true;
                curvePoints[0] = pathCreator.path.GetPointAtDistance(curves[i][curves[i].Length-1]);
                curvePoints[1] = pathCreator.path.GetPointAtDistance(curves[i][Mathf.FloorToInt(curves[i].Length/2)]);
                curvePoints[2] = pathCreator.path.GetPointAtDistance(curves[i][0]);
                //complute drift scale between 0 and 1 and increases with viewer distance to curve center in smooting distance
                if(agentDistance < curves[i][curves[i].Length-1] + curveExpandEntry && agentDistance > curves[i][curves[i].Length-1] + curveExpandEntry - smoothingDistance){
                    driftScale = Mathf.Min(1.0f, ((curves[i][curves[i].Length-1] + curveExpandEntry) - agentDistance) / smoothingDistance);
                }
                else if(agentDistance > curves[i][0] - curveExpandExit && agentDistance < curves[i][0] - curveExpandExit + smoothingDistance){
                    driftScale = Mathf.Min(1.0f, (agentDistance - (curves[i][0] - curveExpandExit)) / smoothingDistance);
                }
            }
        }


        if(!inCurve){
            return 1.0f;
        }

        // //get curve direction from the curve points (-1 for right, 1 for left)
        int curveAngleSign = -(int)Math.Sign(Vector3.Cross(curvePoints[1] - curvePoints[0], curvePoints[2] - curvePoints[1]).y);
        float carAngleSign = (int)Math.Sign(imu.SideSlip);

        // //convert to -1 or 1
        if(carAngleSign != curveAngleSign){
            return -0.01f;
        }
        float rew = Math.Min(1f, (((110f)*(1-driftScale) + 1) / (1f + Mathf.Pow(Mathf.Abs((Mathf.Abs(imu.SideSlip)-45f) / 25f),(2f*4f)))));


        return rew;
    }

    private void cumulativeRewardEvent(float reward){
        totalDistanceText.text = Math.Round(reward).ToString();
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
        if( Input.GetKey(KeyCode.S) ) continuousActionsOut[1] = -1f;
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