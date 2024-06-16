using MathNet.Numerics.LinearAlgebra;
using System.IO;
using UnityEngine;

public class DataloaderRobot : MonoBehaviour {

    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<double> M = Matrix<double>.Build;

    [System.Serializable]
    public class FrameData {
        public int frame;
        public int sequence;
        public int step;
        public float timestamp;
        public Capture[] captures;
    }

    [System.Serializable]
    public class Capture {
        public string type;
        public string id;
        public string description;
        public float[] position;
        public float[] rotation;
        public float[] velocity;
        public float[] acceleration;
        public float[] globalPosition;
        public float[] globalRotation;
        public Annotation[] annotations;
    }

    [System.Serializable]
    public class Annotation {
        public string type;
        public string id;
        public string sensorId;
        public string description;
        public float[] ranges;
        public float[] angles;
        public string[] object_classes;
        public float[] intensities;
    }

    public class RobotData {
        public float timestamp;

        public VehicleState vehicleState;
        public Matrix<double> vehicleStateCovariance;

        public AugmentedObservation[] observations;

        public RobotData(float timestamp, VehicleState state, 
            Matrix<double> stateCovariance, AugmentedObservation[] observations) {

            this.timestamp = timestamp;
            this.vehicleState = state;
            this.vehicleStateCovariance = stateCovariance;
            this.observations = observations;
        }
    }

    public GameObject robotObject;
    public GameObject lidarObject;

    [Tooltip("Button to restart loading data from the beginning")]
    public bool restart = false;

    [Header("Process Noise Error")]
    [Tooltip("Estimated error covariance on the position of the robot")]
    public float errorPos = 0.01f;

    [Tooltip("Estimated error covariance on the angle of the robot (in degrees)")]
    public float errorAngle = 2;

    [Header("Observation Error")]
    [Tooltip("Estimated error covariance on the LIDAR range")]
    public float errorLidarRange = 0.01f;

    [Tooltip("Estimated error covariance on the angle of the LIDAR measures (in degrees)")]
    public float errorLidarAngle = 2;


    private VehicleModel vehicleModel;

    private bool isRunning = false;

    private RobotData currentFrame = null;

    private Vector3 robotPosition;
    private Quaternion robotRotation;

    // Local pose of the LIDAR:
    private Vector3 lidarLocalPosition;
    private Quaternion lidarLocalRotation;

    private float startTime;
    private int currentStep = 0;
    private bool readingComplete = false;

    private bool newFrameAvailable = false;

    // Start is called before the first frame update
    void Start() {
        robotPosition = robotObject.transform.position;
        robotRotation = robotObject.transform.rotation;
        lidarLocalPosition = lidarObject.transform.localPosition;
        lidarLocalRotation = lidarObject.transform.localRotation;

        // Build the covariance matrix representing the error on the robot state:
        Matrix<double> processNoiseError = M.Diagonal(new double[] {
            errorPos * errorPos,
            errorPos * errorPos,
            errorAngle * errorAngle * Mathf.Deg2Rad * Mathf.Deg2Rad });

        // Build the covariance matrix representing the error on the observations:
        Matrix<double> observationError = M.Diagonal(new double[] {
            errorLidarRange * errorLidarRange,
            errorLidarAngle * errorLidarAngle * Mathf.Deg2Rad * Mathf.Deg2Rad,
        });

        // Create a vehicle model with sensorPose = vehiclePose:
        vehicleModel = new VehicleModel(0, 0, 0, processNoiseError, observationError);
    }

    // Update is called once per frame
    void Update() {
        if(isRunning && !readingComplete)
            LoadNextFrame("Assets/data/warehouse", Time.time - startTime);

        robotObject.transform.position = robotPosition;
        robotObject.transform.rotation = robotRotation;
        lidarObject.transform.localPosition = lidarLocalPosition;
        lidarObject.transform.localRotation = lidarLocalRotation;
    }

    public void OnDrawGizmos() {
        if(currentFrame != null) {

            for(int i = 0; i < currentFrame.observations.Length; i++) {
                AugmentedObservation observation = currentFrame.observations[i];
                
                Vector3 direction = Quaternion.AngleAxis(-Mathf.Rad2Deg * observation.theta, Vector3.up) * robotObject.transform.forward;

                Gizmos.color = Color.Lerp(Color.red, Color.white, 1f * i / currentFrame.observations.Length);
                Gizmos.DrawRay(robotObject.transform.position, direction * observation.r);
            }
        }
    }

    public void OnValidate() {
        if (restart) {
            restart = false;
            isRunning = true;

            currentFrame = null;
            startTime = Time.time;
            currentStep = 0;
            
            readingComplete = false;
            newFrameAvailable = false;
        }
    }

    public bool IsNewFrameAvailable() {
        bool tmp = newFrameAvailable;
        newFrameAvailable = false;
        return tmp;
    }

    public RobotData GetCurrentFrame() {
        return currentFrame;
    }

    public VehicleModel GetVehicleModel() {
        return vehicleModel;
    }

    public Pose2D GetSensorPose() {
        // Compute the sensor pose from the current robot data:
        VehicleState vehicleState = currentFrame.vehicleState;

        Vector2 sensorPosition = vehicleModel.GetSensorPosition(vehicleState);
        float sensorAngle = vehicleState.phi;

        return new Pose2D(sensorPosition.x, sensorPosition.y, sensorAngle);
    }

    private void LoadNextFrame(string folder, float currentTime) {
        RobotData nextFrame = currentFrame;

        while(nextFrame == null || nextFrame.timestamp < currentTime) {
            string filename = folder + "/step" + currentStep + ".frame_data.json";
            if(!File.Exists(filename)) {
                readingComplete = true;
                return;
            }

            nextFrame = LoadData(filename);
            currentStep++;
        }

        if(nextFrame != null && nextFrame != currentFrame) {
            currentFrame = nextFrame;
            newFrameAvailable = true;
        }
    }

    private RobotData LoadData(string filename) {
        string text = File.ReadAllText(filename);
        FrameData data = JsonUtility.FromJson<FrameData>(text);

        if (data == null || data.captures == null)
            return null;

        // Extract useful data from the JSON:
        float timestamp = data.timestamp;
        float[] position_data = data.captures[0].position;
        float[] rotation_data = data.captures[0].rotation;
        float[] global_position_data = data.captures[0].globalPosition;
        float[] global_rotation_data = data.captures[0].globalRotation;
        Annotation[] annotations = data.captures[0].annotations;

        if (position_data == null || rotation_data == null || annotations == null
            || global_position_data == null || global_rotation_data == null)
            return null;

        float[] ranges = annotations[0].ranges;
        float[] angles = annotations[0].angles;

        if(ranges == null || angles == null)
            return null;

        // Convert the ranges and angles into observations:
        int count = Mathf.Min(ranges.Length, angles.Length);
        AugmentedObservation[] observations = new AugmentedObservation[count];
        for(int i = 0; i < observations.Length; i++)
            observations[i] = new AugmentedObservation(ranges[i], Mathf.Deg2Rad * angles[i], false);

        robotPosition = ToVector3(global_position_data);
        robotRotation = ToQuaternion(global_rotation_data);
        lidarLocalPosition = ToVector3(position_data);
        lidarLocalRotation = ToQuaternion(rotation_data);

        // Convert the position and rotation into a vehicle state:
        float x = robotPosition.x;
        float y = robotPosition.z;
        float angle = Mathf.Deg2Rad * (90 - robotRotation.eulerAngles.y);
        VehicleState vehicleState = new VehicleState(x, y, angle);
        Debug.Log("Vehicle state: " + vehicleState + "; Robot angle: " + robotRotation.eulerAngles.y);

        return new RobotData(timestamp, vehicleState, vehicleModel.ProcessNoiseError, observations);
    }

    private Vector3 ToVector3(float[] data) {
        return new Vector3(data[0], data[1], data[2]);
    }

    private Quaternion ToQuaternion(float[] data) {
        return new Quaternion(data[0], data[1], data[2], data[3]);
    }
}
