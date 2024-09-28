using UnityEngine;

/// <summary>
/// Script used to record data from Unity for the Python implementation
/// </summary>

public class RobotRecorder : MonoBehaviour {

    [Header("General")]
    [Tooltip("Wait between two measurements from the LIDARs")]
    public float waitBetweenRecords = 0.02f;
    [Tooltip("The folder in which the data should be saved")]
    public string saveFolder = "./Assets/Data";

    [Header("Robot")]
    [Tooltip("GameObject representing the robot")]
    public GameObject robotObject;
    public ControllerParams controllerParams;

    [Header("Lidars")]
    [Tooltip("The list of LIDARs for which data should be recorded")]
    public GameObject[] lidarObjects;
    public LidarParams lidarParams;

    private Lidar[] lidars;
    private RobotController controller;

    private string dataPath;
    private float startTimeRecord;
    private float lastTimeUpdate;
    private int frameNumber = 0;

    void Start() {
        // Instantiate the LIDARs:
        lidars = new Lidar[lidarObjects.Length];
        LidarSetup[] lidarSetups = new LidarSetup[lidarObjects.Length];

        for (int i = 0; i < lidarObjects.Length; i++) {
            lidars[i] = new Lidar(lidarObjects[i], lidarParams.raycastCount, lidarParams.lidarMinRange, lidarParams.lidarMaxRange, i);
            lidarSetups[i] = lidars[i].GetSetup();
        }

        // Instantiate the script to move the robot with arrow keys:
        controller = new RobotController(robotObject, controllerParams);

        Pose2D robotPose = controller.GetRobotRealState().GetPose();

        // Create the config file, representing the initial pose of the robot, and the local
        // pose and descriptions of the LIDARs:
        RobotSetup setup = new RobotSetup(robotPose, lidarSetups);

        // Create a directory to save the data:
        dataPath = WriterUtils.CreateDirectory(saveFolder, "RecordedData");
        Debug.Log("Folder successfully created ! The recorded data will be saved here: " + dataPath);

        // Save the config file in this directory:
        WriterUtils.SaveConfig(setup, dataPath);

        // Record the first frame of data:
        startTimeRecord = Time.time;
        RecordData(robotPose, 0);
        lastTimeUpdate = Time.time;
    }

    // Update is called once per frame
    void Update() {
        float currentTime = Time.time - startTimeRecord;

        // Each frame, update the robot position:
        controller.Update();

        // Every "waitBetweenRecords" seconds, perform a LIDAR scan and use the
        // real state of the robot to build the next robot frame:
        if (currentTime - lastTimeUpdate >= waitBetweenRecords) {
            // Get the real pose of the robot from the controller:
            Pose2D robotPose = controller.GetRobotRealState().GetPose();

            RecordData(robotPose, currentTime);
            lastTimeUpdate = currentTime;
        }
    }

    public void OnDrawGizmos() {
        if (lidars != null) {
            foreach (Lidar lidar in lidars)
                lidar.DrawGizmos(lidarParams.drawRays);
        }
    }

    /// <summary>
    /// Records a frame of data ans save it in the folder specified by this class
    /// </summary>
    private void RecordData(Pose2D robotPose, float timestamp) {
        // Get the observations from the LIDARs:
        LidarData[] lidarsData = new LidarData[lidars.Length];

        for(int i = 0; i < lidars.Length; i++) {
            Observation[] observations = lidars[i].ComputeObservations();

            // Convert the observations into a list of ranges and angles:
            float[] ranges = new float[observations.Length];
            float[] angles = new float[observations.Length];
            for(int j = 0; j < observations.Length; j++) {
                ranges[j] = observations[j].r;
                angles[j] = observations[j].theta;
            }

            lidarsData[i] = new LidarData(i, angles, ranges);
        }

        RobotFrame frame = new RobotFrame(frameNumber, timestamp, robotPose, lidarsData);
        WriterUtils.SaveData(frame, dataPath);

        frameNumber++;
    }
}