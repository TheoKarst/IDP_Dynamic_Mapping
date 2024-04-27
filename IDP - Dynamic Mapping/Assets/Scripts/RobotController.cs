using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Tooltip("Sensor attached to the robot controller, in order to estimate the robot state")]
    public Lidar lidar;

    [Tooltip("Cube prefab used to represent the estimated state of the robot")]
    public GameObject vehicleState;

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
        float x = getRobotX(), y = getRobotY(), phi = getRobotAngle();
        filter = new KalmanFilter(x, y, phi, lidar, L);

        // Initialise the landmarks with some positions (used for testing):
        filter.initLandmarks(lidar.landmarks);
    }

    // Update is called once per frame
    void Update() {
        UpdateRobot();

        if(Time.time - lastTimeMeasure > waitBetweenMeasures) {
            // Get an observation from the LIDAR:
            float landmarkDistance, landmarkAngle;
            (landmarkDistance, landmarkAngle) = lidar.getLandmark();

            Observation observation = new Observation(landmarkDistance, landmarkAngle);
            ModelInputs inputs = new ModelInputs(velocity, Mathf.Deg2Rad * steering);

            // Use the observation to update the robot state estimate:
            ModelState realState = new ModelState(getRobotX(), getRobotY(), getRobotAngle());

            Debug.Log("0. Robot real state: " + realState);
            filter.updateStateEstimate(observation, inputs, Time.time);

            // For debugging, show the error estimate on each landmark and on the robot state:
            filter.resizeLandmarksUsingCovariance(lidar.landmarks);
            filter.resizeStateUsingCovariance(vehicleState.transform);
            lastTimeMeasure = Time.time;
        }
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

    public float getRobotX() {
        return gameObject.transform.position.x;
    }

    public float getRobotY() {
        return gameObject.transform.position.z;
    }

    public float getRobotAngle() {
        return Mathf.Deg2Rad * (90 - gameObject.transform.rotation.eulerAngles.y);
    }
}
