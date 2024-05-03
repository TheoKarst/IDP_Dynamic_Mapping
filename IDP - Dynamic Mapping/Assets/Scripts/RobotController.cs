using System.IO;
using UnityEngine;

public class RobotController : MonoBehaviour {

    [Tooltip("Sensor attached to the robot controller, in order to estimate the robot state")]
    public Lidar lidar;

    public bool writeLogFile = false;

    public float acceleration = 10;
    public float L = 0.3f;

    public float maxSpeed = 0.015f;
    public float maxSteering = 85;

    public float friction = 0.02f;

    public float waitBetweenMeasures = 1;

    public bool drawConfirmedLandmarks, drawPotentialLandmarks, drawObservations;

    public float minDistanceBetweenLandmarks = 1;

    private float velocity = 0;
    private float steering = 0;

    private bool startKalmanFilter = false;
    private KalmanFilter filter;
    private float lastTimeMeasure;

    // Start is called before the first frame update
    void Start() {
        ModelState initialState = getRobotRealState();
        StreamWriter logFile = writeLogFile ? File.CreateText("log_file.csv") : null;

        // Estimates of the different errors in the model:
        ModelParams model = new ModelParams();

        // Dimensions of the model:
        model.L = L;
        model.a = lidar.getLidarA();
        model.b = lidar.getLidarB();

        // Error estimates used in the Kalman Filter:
        model.errorX = model.errorY = 0.4f;
        model.errorPhi = 2 * Mathf.Deg2Rad;
        model.errorR = 0.4f;
        model.errorTheta = 2 * Mathf.Deg2Rad;

        // Also define the minimum distance between landmarks:
        model.minDistanceBetweenLandmarks = minDistanceBetweenLandmarks;

        filter = new KalmanFilter(this, initialState, model, logFile);
    }

    // Update is called once per frame
    void Update() {
        UpdateRobot();

        if(startKalmanFilter && Time.time - lastTimeMeasure > waitBetweenMeasures) {
            UpdateStateEstimate();
            lastTimeMeasure = Time.time;
        }
    }

    public void OnDrawGizmos() {
        if(filter != null)
            filter.drawGizmos(drawConfirmedLandmarks, drawPotentialLandmarks, drawObservations);
    }

    private void UpdateRobot() {
        float h = Time.deltaTime;

        // Update the velocity of the robot:
        if (Input.GetKey(KeyCode.UpArrow))
            velocity += h * acceleration;
        else if (Input.GetKey(KeyCode.DownArrow))
            velocity -= h * acceleration;
        else
            velocity -= h * velocity * friction;

        // Update the steering of the robot:
        if (Input.GetKey(KeyCode.LeftArrow))
            steering = maxSteering;
        else if (Input.GetKey(KeyCode.RightArrow))
            steering = -maxSteering;
        else
            steering = 0;

        // Start to run the Kalman Filter when we start moving the vehicle:
        if (velocity != 0)
            startKalmanFilter = true;

        // Clamp the velocity between boundaries:
        velocity = Mathf.Clamp(velocity, -maxSpeed, maxSpeed);

        // Compute the derivative of the position and orientation of the robot:
        float robotAngle = getRobotAngle();
        float xP = velocity * Mathf.Cos(robotAngle);
        float yP = velocity * Mathf.Sin(robotAngle);
        float angleP = velocity * Mathf.Tan(Mathf.Deg2Rad * steering) / L;

        // Update the position and orientation of the robot, given the previous values:
        gameObject.transform.position += h * new Vector3(xP, 0, yP);
        gameObject.transform.Rotate(Vector3.up, -h * angleP * Mathf.Rad2Deg);
    }

    private void UpdateStateEstimate() {
        ModelInputs inputs = new ModelInputs(velocity, Mathf.Deg2Rad * steering);

        // Get all the observations from the LIDAR:
        Observation[] observations = lidar.getObservations();

        // Use these observations to update the robot state estimate:
        filter.updateStateEstimate(observations, inputs, Time.time);
    }

    public float getRobotX() {
        return gameObject.transform.position.x;
    }

    public float getRobotY() {
        return gameObject.transform.position.z;
    }

    public float getRobotAngle() {
        return Mathf.Deg2Rad * (90 - gameObject.transform.rotation.eulerAngles.y);
    }

    public ModelState getRobotRealState() {
        return new ModelState(getRobotX(), getRobotY(), getRobotAngle());
    }
}
