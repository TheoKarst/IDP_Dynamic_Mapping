using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class DataloaderRobot : Robot {

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

        // List of observations made by each LIDAR on the robot:
        public AugmentedObservation[][] observations;

        public RobotData(float timestamp, VehicleState state,
            Matrix<double> stateCovariance, AugmentedObservation[][] observations) {

            this.timestamp = timestamp;
            this.vehicleState = state;
            this.vehicleStateCovariance = stateCovariance;
            this.observations = observations;
        }
    }

    public class LidarData {
        public Pose2D localPose;

        public Vector3 position;
        public Quaternion rotation;
        public Vector3 localPosition;
        public Quaternion localRotation;

        public AugmentedObservation[] observations;

        public LidarData(Pose2D localPose, Vector3 position, Quaternion rotation, 
            Vector3 localPosition, Quaternion localRotation,
            AugmentedObservation[] observations) {

            this.localPose = localPose;
            this.position = position;
            this.rotation = rotation;
            this.localPosition = localPosition;
            this.localRotation = localRotation;
            this.observations = observations;
        }
    }

    public GameObject robotObject;
    public GameObject rearLidar;
    public GameObject frontLidar;

    public float minObservationRadius = 0.1f;
    public bool drawRays = true;

    [Tooltip("Button to restart loading data from the beginning")]
    public bool restart = false;
    public bool run = false;

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

    private RobotData currentFrame = null;

    private const int FRONT_LIDAR_INDEX = 0;
    private const int REAR_LIDAR_INDEX = 1;

    private float currentTime;
    private int currentStep = 0;
    private bool readingComplete = false;

    private bool newFrameAvailable = false;

    // Start is called before the first frame update
    void Start() {
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

        // Create a vehicle model:
        Pose2D[] lidarPoses = new Pose2D[2];
        lidarPoses[FRONT_LIDAR_INDEX] = GetLocalPose(frontLidar.transform.localPosition, frontLidar.transform.localRotation);
        lidarPoses[REAR_LIDAR_INDEX] = GetLocalPose(rearLidar.transform.localPosition, rearLidar.transform.localRotation);

        vehicleModel = new VehicleModel(lidarPoses, 0, processNoiseError, observationError);
    }

    // Update is called once per frame
    void Update() {
        if (run && !readingComplete) {
            currentTime += Time.deltaTime;
            LoadNextFrame("Assets/data/warehouse", currentTime);
        }
        else if (readingComplete)
            run = false;
    }

    public void OnDrawGizmos() {
        if(drawRays && currentFrame != null) {
            AugmentedObservation[] observations = currentFrame.observations[FRONT_LIDAR_INDEX];
            for (int i = 0; i < observations.Length; i++) {
                AugmentedObservation observation = observations[i];

                Vector3 direction = Quaternion.AngleAxis(-Mathf.Rad2Deg * observation.theta, Vector3.up) * frontLidar.transform.forward;

                Gizmos.color = Color.Lerp(Color.red, Color.white, 1f * i / observations.Length);
                Gizmos.DrawRay(frontLidar.transform.position, direction * observation.r);
            }

            observations = currentFrame.observations[REAR_LIDAR_INDEX];
            for (int i = 0; i < observations.Length; i++) {
                AugmentedObservation observation = observations[i];

                Vector3 direction = Quaternion.AngleAxis(-Mathf.Rad2Deg * observation.theta, Vector3.up) * rearLidar.transform.forward;

                Gizmos.color = Color.Lerp(Color.red, Color.white, 1f * i / observations.Length);
                Gizmos.DrawRay(rearLidar.transform.position, direction * observation.r);
            }
        }
    }

    public void OnValidate() {
        if (restart) {
            restart = false;
            run = false;

            currentFrame = null;
            currentTime = 0;
            currentStep = 0;
            
            readingComplete = false;
            newFrameAvailable = false;
        }
    }

    public override bool IsNewFrameAvailable() {
        bool tmp = newFrameAvailable;
        newFrameAvailable = false;
        return tmp;
    }

    public override RobotData GetCurrentFrame() {
        return currentFrame;
    }

    public override VehicleModel GetVehicleModel() {
        return vehicleModel;
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

        if (data == null || data.captures == null || data.captures.Length < 3)
            return null;

        // Extract useful data from the JSON:
        float timestamp = data.timestamp;

        LidarData rearLidarData = ParseLidarData(data.captures[0], REAR_LIDAR_INDEX);
        LidarData frontLidarData = ParseLidarData(data.captures[2], FRONT_LIDAR_INDEX);

        if(rearLidarData == null || frontLidarData == null)
            return null;

        // Update the transform of the LIDARs:
        frontLidar.transform.position = frontLidarData.position;
        frontLidar.transform.rotation = frontLidarData.rotation;
        rearLidar.transform.position = rearLidarData.position;
        rearLidar.transform.rotation = rearLidarData.rotation;

        // Update the vehicle model accordingly:
        vehicleModel.UpdateLidarPose(FRONT_LIDAR_INDEX, frontLidarData.localPose);
        vehicleModel.UpdateLidarPose(REAR_LIDAR_INDEX, rearLidarData.localPose);

        // Now compute the robot transform from the frontLidar (it would be the same
        // to do so from the rear LIDAR):
        Quaternion rotation = frontLidarData.rotation * Quaternion.Inverse(frontLidarData.localRotation);
        Vector3 position = frontLidarData.position - rotation * frontLidarData.localPosition;

        robotObject.transform.position = position;
        robotObject.transform.rotation = rotation;

        float x = position.x;
        float y = position.z;
        float phi = Mathf.Deg2Rad * (90 - rotation.eulerAngles.y);

        VehicleState vehicleState = new VehicleState(x, y, phi);
        Debug.Log("Vehicle state: " + vehicleState);

        // Stack the observations from both LIDARs into an array:
        AugmentedObservation[][] observations = new AugmentedObservation[2][];
        observations[REAR_LIDAR_INDEX] = rearLidarData.observations;
        observations[FRONT_LIDAR_INDEX] = frontLidarData.observations;

        // Finally return the current frame, with all the data we loaded from the json file:
        return new RobotData(timestamp, vehicleState, vehicleModel.ProcessNoiseError, observations);
    }

    private Vector3 ToVector3(float[] data) {
        return new Vector3(data[0], data[1], data[2]);
    }

    private Quaternion ToQuaternion(float[] data) {
        return new Quaternion(data[0], data[1], data[2], data[3]);
    }

    private LidarData ParseLidarData(Capture capture, int lidarIndex) {
        float[] local_position_data = capture.position;
        float[] local_rotation_data = capture.rotation;
        float[] global_position_data = capture.globalPosition;
        float[] global_rotation_data = capture.globalRotation;
        Annotation[] annotations = capture.annotations;

        if (local_position_data == null || local_rotation_data == null || annotations == null
            || global_position_data == null || global_rotation_data == null)
            return null;

        float[] ranges = annotations[0].ranges;
        float[] angles = annotations[0].angles;

        if (ranges == null || angles == null)
            return null;

        Vector3 localPosition = ToVector3(local_position_data);
        Quaternion localRotation = ToQuaternion(local_rotation_data);
        Vector3 position = ToVector3(global_position_data);
        Quaternion rotation = ToQuaternion(global_rotation_data);
        AugmentedObservation[] observations = BuildObservations(ranges, angles, lidarIndex);

        // Compute the local pose of the LIDAR:
        Pose2D localPose = GetLocalPose(localPosition, localRotation);

        return new LidarData(localPose, position, rotation, localPosition, localRotation, observations);
    }

    private AugmentedObservation[] BuildObservations(float[] ranges, float[] angles, int lidarIndex) {
        int maxCount = Mathf.Min(ranges.Length, angles.Length);

        List<AugmentedObservation> observations = new List<AugmentedObservation>();
        for(int i = 0; i < maxCount; i++) {
            if (ranges[i] >= minObservationRadius)
                observations.Add(new AugmentedObservation(ranges[i], -Mathf.Deg2Rad * angles[i], lidarIndex, false));
        }

        return observations.ToArray();
    }

    private Pose2D GetLocalPose(Vector3 localPosition, Quaternion localRotation) {
        float x = localPosition.z;
        float y = -localPosition.x;
        float angle = -localRotation.eulerAngles.y * Mathf.Deg2Rad;

        return new Pose2D(x, y, angle);
    }
}
