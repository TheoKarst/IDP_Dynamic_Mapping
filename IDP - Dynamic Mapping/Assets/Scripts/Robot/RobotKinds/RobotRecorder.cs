using UnityEngine;

public class RobotRecorder : MonoBehaviour {

    [Header("General")]
    [Tooltip("Wait between two measurements from the LIDARs")]
    public float waitBetweenRecords = 0.02f;
    public string saveFolder = "./Assets/Data";

    [Header("Robot")]
    public GameObject robotObject;
    public ControllerParams controllerParams;

    [Header("Lidars")]
    public GameObject[] lidarObjects;

    public int raycastCount = 500;
    public float lidarMinRange = 0;
    public float lidarMaxRange = 10;

    public bool drawRays = true;

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
            lidars[i] = new Lidar(lidarObjects[i], raycastCount, lidarMinRange, lidarMaxRange, i);
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
                lidar.DrawGizmos(drawRays);
        }
    }

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