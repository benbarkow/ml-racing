using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehiclePhysics;

public class DriftAgent: Agent
{
    // Environment Settings
    public float MaxDistFromPath = 5f;
    public int NumNextKnownWaypoints = 5;
    public float SteerSpeedDegPerSec = 100f;
    public GameObject PointPopupPrefab;

    public DriftTrajectory[] trajectories;


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
        if(traj) Destroy(traj.gameObject);
        traj = Instantiate(trajectories[Random.Range(0,trajectories.Length)]);
        traj.reset(MaxDistFromPath);

        //reset the agent to face the current waypoint
        this.transform.position = traj.getWaypointRelativeToCurrent(-1);
        this.transform.LookAt(traj.getWaypointRelativeToCurrent(0));
        VehiclePhysics.VPResetVehicle.ResetVehicle(VPbase, 0.5f, true);
        VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 2);
        rb.isKinematic = false;
        imu.rBody.isKinematic = false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localVel = imu.LocalVelocity;
        Vector2 localVelXZ = new Vector2(localVel.x, localVel.z);
        sensor.AddObservation(localVelXZ / 300f);

        sensor.AddObservation(-rb.angularVelocity.y / 10f);

        sensor.AddObservation(Mathf.Abs(imu.SideSlip) / 180f);

        Vector3 nextWP;
        Vector2 nextWP2;
        float maxWPdist = NumNextKnownWaypoints * traj.WaypointSpacing;
        for (int i = 0; i < NumNextKnownWaypoints; i++)
        {
            nextWP = transform.InverseTransformPoint(traj.getWaypointRelativeToCurrent(i));
            //Debug.DrawLine(car.transform.position, RaceTrack.getWaypointRelativeToCurrent(i), Color.green);
            nextWP2 = new Vector2(nextWP.x / maxWPdist, nextWP.z / maxWPdist); //normalize to [0,1]

            sensor.AddObservation(nextWP2);
        }

        sensor.AddObservation(VPcontrol.data.Get(Channel.Vehicle, VehicleData.EngineRpm) / (1000.0f * VPcontrol.engine.maxRpm ));
        sensor.AddObservation(traj.getDistOfCarToPath(transform) / MaxDistFromPath);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        //if( VPcontrol.data.Get(Channel.Vehicle, VehicleData.EngineRpm) / 1000.0f > 4500f ) VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 2);
        //if( VPcontrol.data.Get(Channel.Vehicle, VehicleData.EngineRpm) / 1000.0f < 2000f ) VPcontrol.data.Set(Channel.Input, InputData.ManualGear, 1);

        if (actionBuffers.ContinuousActions[0] == 0f){
            //VPinput.externalSteer = SmoothSteering(imu.SideSlip / VPcontrol.steering.maxSteerAngle); //Gyro
            VPinput.externalSteer = SmoothSteering(-rb.angularVelocity.y * 0.030516f);        //mapping to degrees per second);
        }
        else{
            VPinput.externalSteer = SmoothSteering(actionBuffers.ContinuousActions[0]);
        }
        

        //Debug.Log(rb.angularVelocity.y);

        VPinput.externalThrottle = PathMathSupports.Remap(actionBuffers.ContinuousActions[1], -1f, 1f, 0f, 1f);
        //VPinput.externalThrottle += actionBuffers.ContinuousActions[1] * 0.05f;
        //VPinput.externalThrottle = Mathf.Clamp(VPinput.externalThrottle, 0f, 1f);
        //if (VPinput.externalThrottle <= 0.1){
            //VPinput.externalHandbrake = 1;
        //}else{
            //VPinput.externalHandbrake = 0f;
        //}


        if (traj.getDistOfCarToPath(this.transform) >= MaxDistFromPath) //|| Mathf.Abs(imu.SideSlip) > 110f)
        {
            //print("too far from path, resetting..." + traj.getDistOfCarToPath(this.transform));
            //SetReward(-10f);
            EndEpisode();
        }


        traj.drawWpLines(this.transform, NumNextKnownWaypoints);

        float rew = 0.0f;//-0.0001f;

        if(traj.getDistOfCarToCurrWpOrthgLine(this.transform) <= 0.2f){
            rew += 1/20f * (MaxDistFromPath - traj.getDistOfCarToOrthgLineWhilePassing(this.transform)) / MaxDistFromPath;//1/20f *

            int desiredAngleSign = PathMathSupports.PointLeftOrRightFromLine(traj.getWaypointRelativeToCurrent(-1), traj.getWaypointRelativeToCurrent(1), traj.getWaypointRelativeToCurrent(0));
            int currentAngleSign = (int)Mathf.Sign(imu.SideSlip);

            //if(Mathf.Abs(imu.SideSlip) < 90 && Mathf.Abs(imu.SideSlip) > 25 || desiredAngleSign == 0){
                float PathCurvatureFactor = 1f;

                //if(desiredAngleSign != 0 && desiredAngleSign == currentAngleSign) PathCurvatureFactor = 1f;//print("desired");
                //else if(desiredAngleSign != 0 && desiredAngleSign != currentAngleSign) PathCurvatureFactor = -0.01f;//print("not desired");
                //else PathCurvatureFactor = 1f;//print("dont care");

                //rew += 18f/20f * (Mathf.Abs(imu.SideSlip) / 90f) * PathCurvatureFactor;
                rew += 19f/20f * PathCurvatureFactor * (1f / (1f + Mathf.Pow(Mathf.Abs((Mathf.Abs(imu.SideSlip) - 45f) / 25f),(2f*4f))));
                //https://www.wolframalpha.com/input/?i=plot+1%2F+%281+%2B+abs%28%28x-c%29%2Fa%29%5E%282b%29%29+for+a%3D15%2Cb%3D1.5%2Cc%3D50+from+x%3D0+to+90
            //} 

            //quick and dirty way of display current tire friction. I should probably make a nice GUI display on screen...
                //Instantiate(PointPopupPrefab, traj.transform.position, Camera.main.transform.rotation).GetComponent<pointPopup>().display(VPcontrol.tireFriction.settings.peak.y);
            

        SetReward(rew);
        //show reward above car
        Instantiate(PointPopupPrefab, transform.position, Camera.main.transform.rotation).GetComponent<pointPopup>().display(rew);
        //print(rew);
        traj.switchToNextWaypoint();
        }
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
