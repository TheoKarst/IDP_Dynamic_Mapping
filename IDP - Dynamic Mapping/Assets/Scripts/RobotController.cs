using System.IO;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Tooltip("Sensor attached to the robot controller, in order to estimate the robot state")]
    public Lidar lidar;

    public bool writeLogFile = false;

    public float acceleration = 10;
    public float L = 0.3f;

    public float maxSpeed = 0.015f;
    public float maxSteering = 85;

    public float friction = 0.02f;

    public float waitBetweenMeasures = 1;

    private float velocity = 0;
    private float steering = 0;

    private KalmanFilter filter;
    private float lastTimeMeasure;

    // Start is called before the first frame update
    void Start() {
        ModelState initialState = getRobotRealState();
        ModelParams model = new ModelParams(L, lidar.getLidarA(), lidar.getLidarB());
        StreamWriter logFile = writeLogFile ? File.CreateText("log_file.csv") : null;

        filter = new KalmanFilter(this, initialState, model, logFile);

        // Initialise the landmarks with some positions (used for testing):
        // filter.initLandmarks(lidar.landmarks);
    }

    // Update is called once per frame
    void Update() {
        UpdateRobot();

        if(Time.time - lastTimeMeasure > waitBetweenMeasures) {
            UpdateStateEstimate();
            lastTimeMeasure = Time.time;
        }
    }

    public void OnDrawGizmos() {
        if(filter != null)
            filter.drawGizmos();
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
        // Get an observation from the LIDAR:
        float landmarkDistance, landmarkAngle;
        (landmarkDistance, landmarkAngle) = lidar.getLandmark();

        Observation observation = new Observation(landmarkDistance, landmarkAngle);
        ModelInputs inputs = new ModelInputs(velocity, Mathf.Deg2Rad * steering);

        // Use the observation to update the robot state estimate:
        filter.updateStateEstimate(observation, inputs, Time.time);
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
