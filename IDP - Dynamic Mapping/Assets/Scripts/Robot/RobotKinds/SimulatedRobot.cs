using UnityEngine;

/// <summary>
/// Differently from the KalmanRobot, this kind of robot uses the real pose of the robot (given by Unity)
/// for the mapping. This is thus faster and more accurate than the KalmanRobot, but also less realistic
/// since in reality, the real pose of the robot is unknown.
/// 
/// This script can however be used if the focus is on the mapping part and not the localization.
/// </summary>

public class SimulatedRobot : Robot {

    [Header("General")]
    [Tooltip("Wait before performing a new lidar scan, position estimate and map update")]
    public float waitBetweenUpdates = 0.02f;

    [Header("Robot")]
    [Tooltip("GameObject representing the robot")]
    public GameObject robotObject;
    public ControllerParams controllerParams;

    [Header("Lidar")]
    [Tooltip("GameObject representing the LIDAR")]
    public GameObject lidarObject;
    public LidarParams lidarParams;

    private Lidar lidar;
    private VehicleModel vehicleModel;
    private RobotController controller;

    // Last time we performed a complete update (lidar measures + robot state estimate + map update):
    private float lastTimeUpdate = 0;

    private bool newFrameAvailable = false;
    private RobotData currentFrame = null;

    void Start() {
        // Instantiate the LIDAR:
        lidar = new Lidar(lidarObject, lidarParams.raycastCount, lidarParams.lidarMinRange, lidarParams.lidarMaxRange, 0);

        // Instantiate the model we are going to use for the robot:
        LidarSetup[] lidarSetups = new LidarSetup[] { lidar.GetSetup() };
        vehicleModel = new VehicleModel(lidarSetups, controllerParams.L);

        // Instantiate the script to move the robot with arrow keys:
        controller = new RobotController(robotObject, controllerParams);
    }

    // Update is called once per frame
    void Update() {
        float currentTime = Time.time;

        // Each frame, update the robot position:
        controller.Update();

        // Every "waitBetweenUpdates" seconds, perform a LIDAR scan and use the
        // real state of the robot to build the next robot frame:
        if (currentTime - lastTimeUpdate >= waitBetweenUpdates) {
            // Use raycasting to compute observations from the LIDAR:
            Observation[][] observations = new Observation[][] {
                lidar.ComputeObservations()
            };

            // Get the real state of the robot from the controller:
            VehicleState vehicleState = controller.GetRobotRealState();

            // Create a new data frame to represent the current state of the robot:
            currentFrame = new RobotData(currentTime, vehicleState, vehicleModel.ProcessNoiseError, observations);
            newFrameAvailable = true;

            lastTimeUpdate = currentTime;
        }
    }

    public void OnDrawGizmos() {
        if (lidar != null)
            lidar.DrawGizmos(lidarParams.drawRays);
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
}