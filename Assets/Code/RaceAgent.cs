using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehiclePhysics;
using PathCreation;

public class RaceAgent : Agent
{
    private Rigidbody rb;    
    private VehiclePhysics.VPStandardInput VPinput;
    private VehiclePhysics.VPVehicleController VPcontrol;
    private VehiclePhysics.VehicleBase VPbase;
    private IntertialMeasurementUnit imu;
    public float SteerSpeedDegPerSec = 100f;
    private Transform startTransform;
    private float startDistanceOnPath;
    private float CurrentSteerDirection;
    private float prevDistanceCovered;

    public PathCreator pathCreator;

    void Start()
    {
        //cache all later requiered components
        VPcontrol = GetComponent<VehiclePhysics.VPVehicleController>();
        VPinput = GetComponent<VehiclePhysics.VPStandardInput>();
        VPbase = GetComponent<VehiclePhysics.VehicleBase>();
        rb = GetComponent<Rigidbody>();
        imu = GetComponentInChildren<IntertialMeasurementUnit>();
        prevDistanceCovered = 0.0f;
    }

    public override void OnEpisodeBegin()
    {
        //reset to start position
        // This resets the vehicle and 'drops' it from a height of 0.5m (so that it does not clip into the ground and get stuck)
        VehiclePhysics.VPResetVehicle.ResetVehicle(VPbase, 0.5f, true);

        float maxPathLength = pathCreator.path.length;
        //compute a random distance along the path
        startDistanceOnPath = Random.Range(0.0f, maxPathLength);

        this.transform.position = pathCreator.path.GetPointAtDistance(startDistanceOnPath);
        //increase y position to avoid clipping into the ground
        this.transform.position = new Vector3(this.transform.position.x, this.transform.position.y + 0.5f, this.transform.position.z);
        this.transform.rotation = pathCreator.path.GetRotationAtDistance(startDistanceOnPath);
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
        float reward = 0.0f;
        float maxStepDistance = 0.2f;
        float maxAngle = 45.0f;
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
        float stepDistanceCovered = distanceCovered - prevDistanceCovered;
        prevDistanceCovered = distanceCovered;
        //set reward and norm with maxStepDistance with max reward of 1
        reward = Mathf.Min(stepDistanceCovered / maxStepDistance, 1.0f);

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

        //set negative reward proportional to angle
        reward -= Mathf.Min(angle / maxAngle, 1.0f);
        if(angle > maxAngle){
            SetReward(-1.0f);
            EndEpisode();
        }
        else{
            SetReward(reward);
        }

        //calculate distance to center of the road
        Vector3 closestPointOnPath = pathCreator.path.GetPointAtDistance(distanceOnPath);
        float distanceToCenter = Vector3.Distance(this.transform.position, closestPointOnPath);
        // Debug.Log("distance to road center: " + distanceToCenter.ToString());
        //if car runs off the road
        if(distanceToCenter > 2.5){
            SetReward(-0.1f);
        }
        if(distanceToCenter > 3.0){
            SetReward(-1.0f);
            EndEpisode();
        }
        //log reward this step
        // Debug.Log("reward: " + reward.ToString());
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