using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehiclePhysics;

public class CarAgent : Agent
{
    public Transform finishLine;

    private Rigidbody rb;    
    private VehiclePhysics.VPStandardInput VPinput;
    private VehiclePhysics.VPVehicleController VPcontrol;
    private VehiclePhysics.VehicleBase VPbase;
    private IntertialMeasurementUnit imu;
    private Vector3 startPosition;

    void Start()
    {
        //cache all later requiered components
        VPcontrol = GetComponent<VehiclePhysics.VPVehicleController>();
        VPinput = GetComponent<VehiclePhysics.VPStandardInput>();
        VPbase = GetComponent<VehiclePhysics.VehicleBase>();
        rb = GetComponent<Rigidbody>();
        imu = GetComponentInChildren<IntertialMeasurementUnit>();
        startPosition = this.transform.localPosition;
    }

    public override void OnEpisodeBegin()
    {
        //reset to start position
        Debug.Log("start position: " + startPosition);
        // This resets the vehicle and 'drops' it from a height of 0.5m (so that it does not clip into the ground and get stuck)
        VehiclePhysics.VPResetVehicle.ResetVehicle(VPbase, 0.5f, true);
        this.transform.localPosition = startPosition;
        rb.isKinematic = false;
        imu.rBody.isKinematic = false;

        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float maxDistance = 50f;
        float distanceToFinish = Vector3.Dot(this.transform.forward, finishLine.localPosition - this.transform.localPosition);
        sensor.AddObservation((maxDistance - distanceToFinish)/maxDistance);
        sensor.AddObservation(rb.velocity.z);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Debug.Log(actionBuffers.ContinuousActions[0]);
        if(actionBuffers.ContinuousActions[0] > 0f){
            VPinput.externalThrottle = actionBuffers.ContinuousActions[0];
            VPinput.externalBrake = 0f;
        }
        else{
            VPinput.externalBrake = -actionBuffers.ContinuousActions[0];
            VPinput.externalThrottle = 0f;
        }

        float maxDistance = 50f;
        // // example: if the overall velocity vector magnitude (i.e. x,y,z together) is greater than 10m/s, end the episode
        float distanceToFinish = Vector3.Dot(this.transform.forward, finishLine.localPosition - this.transform.localPosition);
        Debug.Log(distanceToFinish);

        if (distanceToFinish < -10f){
            SetReward(-0.2f);
            EndEpisode();
        }
        else if (distanceToFinish < 0f && rb.velocity.z < 0.5f){
            SetReward(1f);
            // EndEpisode();
        }
        else {
            SetReward(((maxDistance - distanceToFinish)/maxDistance)*0.1f);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        //steer
        // continuousActionsOut[0] = 0f;
        // if( Input.GetKey(KeyCode.D) ) continuousActionsOut[0] = 1f;
        // if( Input.GetKey(KeyCode.A) ) continuousActionsOut[0] = -1f;
        
        //throttle
        continuousActionsOut[0] = 0f;
        if( Input.GetKey(KeyCode.W) ) continuousActionsOut[0] = 1f;
        if( Input.GetKey(KeyCode.S) ) continuousActionsOut[0] = -1f;
    }
}
