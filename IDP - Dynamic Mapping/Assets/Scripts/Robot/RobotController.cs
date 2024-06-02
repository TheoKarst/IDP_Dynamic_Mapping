using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour {

    [Header("Robot")]

    [Tooltip("Sensor attached to the robot controller, in order to estimate the robot state")]
    public Lidar lidar;

    // Used for debugging: this is used to force forward or backward motion:
    public bool forceForward = false;
    public bool forceBackward = false;

    public float acceleration = 10;
    public float L = 0.3f;

    public float maxSpeed = 0.015f;
    public float maxSteering = 85;

    public float friction = 0.02f;

    [Header("Landmarks")]
    [Tooltip("The maximum number of landmarks we can use to update the robot " +
        "state estimate (more landmarks means more precision, but is also slower)")]
    public int maxLandmarksPerUpdate = 5;
    public float minDistanceBetweenLandmarks = 1;

    public bool drawConfirmedLandmarks, drawPotentialLandmarks, drawObservations;

    [Header("Other")]
    public bool writeLogFile = false;
    public float waitBetweenMeasures = 1;

    private float velocity = 0;
    private float steering = 0;

    private VehicleModel vehicleModel;
    private KalmanFilter filter;
    private float lastTimeMeasure = 0;

    // Start is called before the first frame update
    void Start() {
        VehicleState initialState = GetRobotRealState();

        // Instantiate a vehicle model:
        vehicleModel = new VehicleModel(lidar, L, maxSpeed, Mathf.Deg2Rad * maxSteering, waitBetweenMeasures);

        Logger kalmanLogger = writeLogFile ? new Logger(false, "log_file.csv") : new Logger(false);
        filter = new KalmanFilter(this, initialState, vehicleModel, maxLandmarksPerUpdate, minDistanceBetweenLandmarks, kalmanLogger);
    }

    // Update is called once per frame
    void Update() {
        UpdateRobot();

        // Wait before updating the state estimate, to have a realistic simulation. Otherwise, the
        // prediction step of the Kalman Filter will always be 100% accurate, which is not realistic
        if(Time.time - lastTimeMeasure > waitBetweenMeasures) {
            UpdateStateEstimate();
            lastTimeMeasure = Time.time;
        }
    }

    public void OnDrawGizmos() {
        if(filter != null)
            filter.DrawGizmos(drawConfirmedLandmarks, drawPotentialLandmarks, drawObservations);
    }

    private void UpdateRobot() {
        float h = Time.deltaTime;

        // Update the velocity of the robot:
        if (forceForward || Input.GetKey(KeyCode.UpArrow))
            velocity += h * acceleration;
        else if (forceBackward || Input.GetKey(KeyCode.DownArrow))
            velocity -= h * acceleration;
        else
            velocity -= h * velocity * friction;

        // Clamp the velocity between boundaries:
        velocity = Mathf.Clamp(velocity, -maxSpeed, maxSpeed);

        // Update the steering of the robot:
        if (Input.GetKey(KeyCode.LeftArrow))
            steering = maxSteering;
        else if (Input.GetKey(KeyCode.RightArrow))
            steering = -maxSteering;
        else
            steering = 0;

        // Compute the derivative of the position and orientation of the robot:
        float robotAngle = GetRobotAngle();
        float xP = velocity * Mathf.Cos(robotAngle);
        float yP = velocity * Mathf.Sin(robotAngle);
        float angleP = velocity * Mathf.Tan(Mathf.Deg2Rad * steering) / L;

        // Update the position and orientation of the robot, given the previous values:
        gameObject.transform.position += h * new Vector3(xP, 0, yP);
        gameObject.transform.Rotate(Vector3.up, -h * angleP * Mathf.Rad2Deg);
    }

    private void UpdateStateEstimate() {
        ModelInputs inputs = new ModelInputs(velocity, Mathf.Deg2Rad * steering);

        // Perform a LIDAR scan:
        lidar.PerformLidarScan();

        // Get the observations from the LIDAR, that are good landmark candidates:
        List<Observation> observations = lidar.GetLandmarkCandidates();
        
        // Use these observations to update the robot state estimate:
        filter.UpdateStateEstimate(observations, inputs, Time.time);
        // LogVehicleState();
    }

    public float GetRobotX() {
        return gameObject.transform.position.x;
    }

    public float GetRobotY() {
        return gameObject.transform.position.z;
    }

    public float GetRobotAngle() {
        return Mathf.Deg2Rad * (90 - gameObject.transform.rotation.eulerAngles.y);
    }

    public VehicleModel GetVehicleModel() {
        return vehicleModel;
    }

    public (VehicleState, Matrix<float>) GetRobotStateEstimate() {
        return filter.GetStateEstimate();
    }

    // For log purposes:
    public VehicleState GetRobotRealState() {
        return new VehicleState(GetRobotX(), GetRobotY(), GetRobotAngle());
    }

    public void LogVehicleState() {
        (VehicleState state, Matrix<float> covariance) = filter.GetStateEstimate();

        string x = Utils.ScientificNotation(state.x);
        string y = Utils.ScientificNotation(state.y);
        string phi = Utils.ScientificNotation(Mathf.Rad2Deg * state.phi);

        string covX = Utils.ScientificNotation(Mathf.Sqrt(covariance[0, 0]));
        string covY = Utils.ScientificNotation(Mathf.Sqrt(covariance[1, 1]));
        string covPhi = Utils.ScientificNotation(Mathf.Rad2Deg * Mathf.Sqrt(covariance[2, 2]));

        Debug.Log("Vehicle state: (x, y, phi) = (" + x + "+-" + covX + ", " +
            y + "+-" + covY + ", " + phi + "+-" + covPhi);
    }
}
