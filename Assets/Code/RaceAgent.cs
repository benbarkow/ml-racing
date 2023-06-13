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

    public float SteerSpeedDegPerSec = 100f;
    public PathCreator pathCreator;
    public Camera cam;

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

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (actionBuffers.ContinuousActions[0] == 0f){
            //VPinput.externalSteer = SmoothSteering(imu.SideSlip / VPcontrol.steering.maxSteerAngle); //Gyro
            VPinput.externalSteer = SmoothSteering(-rb.angularVelocity.y * 0.030516f);        //mapping to degrees per second);
        }
        else{
            VPinput.externalSteer = SmoothSteering(actionBuffers.ContinuousActions[0]);
        }
        VPinput.externalThrottle = PathMathSupports.Remap(actionBuffers.ContinuousActions[1], 0f, 1f, 0f, 1f);

        //calculate reward
        int pathDirection = 1;
        float distReward = 0.0f;
        float maxStepDistance = 0.2f;
        float maxAngle = 60.0f;
        //calculate distance traveled along the path from the start transform
        float distanceCovered = 0.0f;
        float distanceOnPath = pathCreator.path.GetClosestDistanceAlongPath(this.transform.position);
        if(distanceOnPath < startDistanceOnPath){
            // Debug.Log("distance covered is negative: "a + distanceOnPath.ToString() + " < " + startDistanceOnPath.ToString());
            distanceCovered = pathCreator.path.length - startDistanceOnPath + distanceOnPath;
        }
        else{
            distanceCovered = distanceOnPath - startDistanceOnPath;
        }
        if(distanceCovered < prevDistanceCovered){
            pathDirection = -1;
        }
        float stepDistanceCovered = (Mathf.Abs(distanceCovered - prevDistanceCovered));


        prevDistanceCovered = distanceCovered;
        //set reward and norm with maxStepDistance with max reward of 1
        distReward = Mathf.Min(stepDistanceCovered / maxStepDistance, 1.0f);
        // Debug.Log("distance reward: " + distReward.ToString());

        //get direction of travel
        Vector3 direction = rb.velocity.normalized;
        Vector2 direction2D = new Vector2(direction.x, direction.z);
        // Debug.Log("direction: " + direction2D.ToString());
        //get angle between direction and path tangent
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
        float angleReward = Mathf.Max(1-(angle / maxAngle), 0.0f);
        angleReward = angleReward * velocityReward;
        // Debug.Log("angle reward: " + angleReward.ToString());
        if(angle > maxAngle && StepCount > 20){
            rewardEvent(-1.0f, pathDirection);
            EndEpisode();
            return;
        }

        //calculate steer delta 
        float maxSteerDelta = 0.044444f;
        float steerDelta = Mathf.Min(Mathf.Abs(CurrentSteerDirection - PreviousSteerDirection)/maxSteerDelta, 1.0f);
        float steerDeltaPenalty = Mathf.Pow(steerDelta, 4) * -0.3f;

        PreviousSteerDirection = CurrentSteerDirection;

        //calculate distance to center of the road
        Vector3 closestPointOnPath = pathCreator.path.GetPointAtDistance(distanceOnPath);
        float distanceToCenter = Vector3.Distance(this.transform.position, closestPointOnPath);
        // Debug.Log("distance to road center: " + distanceToCenter.ToString());
        //if car runs off the road
        float centerDistRewardNegative = Mathf.Max((1-Mathf.Pow(distanceToCenter/3, 8))-1, -1.0f);
        // Debug.Log("center distance reward: " + centerDistRewardNegative.ToString());
        if(distanceToCenter > 3.0 && StepCount > 20){
            rewardEvent(-1.0f, pathDirection);
            SetReward(-1.0f);
            EndEpisode();
            return;
        }

        //weight rewards with distance as 7/8 and angle as 1/8
        float reward = (distReward * 7 + angleReward) / 8;
        //add negative reward for running off the road
        reward += centerDistRewardNegative;
        reward += steerDeltaPenalty;
        SetReward(reward);
        //log reward this step
        // Debug.Log("reward: " + reward.ToString());
        // Debug.Log("total reward: " + GetCumulativeReward().ToString());
        // rewardEvent(angleReward, pathDirection);
        rewardEvent(reward, pathDirection);
        distanceCoveredUpdate(pathDirection == 1 ? distanceCovered : pathCreator.path.length - distanceCovered);
    }

    private void distanceCoveredUpdate(float distanceCovered){
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