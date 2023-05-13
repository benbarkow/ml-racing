using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehiclePhysics;

public class BasicCarAgent: Agent
{
    // Environment Settings
    public float SteerSpeedDegPerSec = 100f;


    // Private Variables
    private Rigidbody rb;    
    private DriftTrajectory traj;//currently selected trajectory
    private VehiclePhysics.VPStandardInput VPinput;
    private VehiclePhysics.VPVehicleController VPcontrol;
    private VehiclePhysics.VehicleBase VPbase;
    private IntertialMeasurementUnit imu;
    private float CurrentSteerDirection;




    void Start()
    {
        //cache all later requiered components
        VPcontrol = GetComponent<VehiclePhysics.VPVehicleController>();
        VPinput = GetComponent<VehiclePhysics.VPStandardInput>();
        VPbase = GetComponent<VehiclePhysics.VehicleBase>();
        rb = GetComponent<Rigidbody>();
        imu = GetComponentInChildren<IntertialMeasurementUnit>();
    }

    public override void OnEpisodeBegin()
    {
        // This resets the vehicle and 'drops' it from a height of 0.5m (so that it does not clip into the ground and get stuck)
        VehiclePhysics.VPResetVehicle.ResetVehicle(VPbase, 0.5f, true);
        rb.isKinematic = false;
        imu.rBody.isKinematic = false;
        
        // go into first gear
        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // example observation: RPM (normalized between 0.0-1.0)
        float currentRPM = VPcontrol.data.Get(Channel.Vehicle, VehicleData.EngineRpm);
        float maxRPM = (1000.0f * VPcontrol.engine.maxRpm );
        sensor.AddObservation( currentRPM / maxRPM );

        // example observation: current speed (normalized between 0.0-1.0)
        float currentSpeed = rb.velocity.magnitude;
        float maxSpeed = 10f; // m/s, because when larger than 10 the episode ends (see below)
        sensor.AddObservation( currentSpeed / maxSpeed );
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // set steering and throttle
        VPinput.externalSteer = SmoothSteering(actionBuffers.ContinuousActions[0]);
        VPinput.externalThrottle = actionBuffers.ContinuousActions[1];

        // example: if the overall velocity vector magnitude (i.e. x,y,z together) is greater than 10m/s, end the episode
        if (rb.velocity.magnitude > 10f){
            EndEpisode();
        }

        float rew = 0.0f;
        if (rb.velocity.magnitude > 1f && rb.velocity.magnitude < 10f){
            rew = rb.velocity.magnitude; // current reward = current speed   :)
        }

        AddReward(rew);
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
