using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;
using static DataloaderRobot;

public class KalmanRobot : Robot {

    [Header("General")]
    [Tooltip("The manager will be used to know if a landmark candidate is static or dynamic")]
    public RobotManager manager;

    [Tooltip("Wait before performing a new lidar scan, position estimate and map update")]
    public float waitBetweenUpdates = 0.02f;

    [Header("Robot")]
    public GameObject robotObject;
    public ControllerParams controllerParams;

    [Header("Lidar")]
    public GameObject lidarObject;

    public int raycastCount = 500;
    public float lidarMinRange = 0;
    public float lidarMaxRange = 10;
    public float douglasPeuckerEpsilon = 0.2f;

    public bool drawRays = true;
    public bool drawCorners = true;

    [Header("Kalman Filter")]
    public KalmanParams kalmanParams;

    private Lidar lidar;
    private VehicleModel vehicleModel;
    private RobotController controller;
    private KalmanFilter kalmanFilter;

    // Last time we performed a complete update (lidar measures + robot state estimate + map update):
    private float lastTimeUpdate = 0;

    private bool newFrameAvailable = false;
    private RobotData currentFrame = null;

    // State estimate of the robot, from the Kalman Filter:
    private VehicleState robotStateEstimate;

    void Start() {
        // Instantiate the LIDAR:
        lidar = new Lidar(lidarObject, raycastCount, lidarMinRange, lidarMaxRange, 0);

        // Instantiate the model we are going to use for the robot:
        LidarSetup[] lidarSetups = new LidarSetup[] { lidar.GetSetup() };
        vehicleModel = new VehicleModel(lidarSetups, controllerParams.L, controllerParams.maxSpeed,
            Mathf.Deg2Rad * controllerParams.maxSteering, waitBetweenUpdates);

        // Instantiate the script to move the robot with arrow keys:
        controller = new RobotController(robotObject, controllerParams);

        // Instantiate the Kalman Filter to estimate the robot position. We initialize
        // it with the real pose of the robot:
        robotStateEstimate = controller.GetRobotRealState();
        Logger kalmanLogger = kalmanParams.writeLogFile ? new Logger(false, "kalman_estimate.csv") : new Logger(false);
        kalmanFilter = new KalmanFilter(this, robotStateEstimate, vehicleModel, kalmanParams, kalmanLogger, 0);
    }

    // Update is called once per frame
    void Update() {
        float currentTime = Time.time;

        // Each frame, update the robot position:
        controller.Update();

        // Perform a whole update less often (in reality, the robot state in continuous,
        // while the LIDAR measures, the state estimate and map update are discrete):
        if (currentTime - lastTimeUpdate >= waitBetweenUpdates) {
            // Use raycasting to compute observations from the LIDAR:
            Observation[][] observations = new Observation[][] {
                lidar.ComputeObservations()
            };
            float maxRange = lidar.GetSetup().max_range;

            // Get the current inputs of the robot:
            ModelInputs inputs = controller.GetModelInputs();

            Profiler.BeginSample("Extract landmark candidates");
            // Get the observations from the LIDAR that are good landmarks candidates:
            int[] filteredObservations = LidarUtils.DouglasPeucker(observations[0], douglasPeuckerEpsilon);
            List<int> landmarkCandidates = LidarUtils.ExtractConvexCorners(observations[0], maxRange, filteredObservations, 220);

            // Use the static landmark candidates to update our state estimate. To find the static
            // landmarks, we need a state estimate. For that we can use the vehicle model and the
            // previous state estimate. Using these landmarks, the Kalman Filter will then provide
            // a better estimate of the real state of the robot:
            VehicleState statePrediction = vehicleModel.PredictCurrentState(robotStateEstimate, inputs, currentTime - lastTimeUpdate);
            List<Observation> staticLandmarkCandidates = LidarUtils.GetStaticObservations(observations[0], landmarkCandidates, manager.GetWorldModel(), vehicleModel, statePrediction);
            Profiler.EndSample();

            Profiler.BeginSample("Kalman Update");
            // Use these observations to update the robot state estimate:
            kalmanFilter.UpdateStateEstimate(staticLandmarkCandidates, inputs, currentTime);
            VehicleState vehicleState = kalmanFilter.GetStateEstimate();
            Matrix<double> vehicleStateCovariance = kalmanFilter.GetStateCovarianceEstimate();
            Profiler.EndSample();

            // Create a new data frame to represent the current state of the robot:
            currentFrame = new RobotData(currentTime, vehicleState, vehicleStateCovariance, observations);
            newFrameAvailable = true;

            lastTimeUpdate = currentTime;
        }
    }

    public void OnDrawGizmos() {
        if(lidar != null) 
            lidar.DrawGizmos(drawRays);

        if (kalmanFilter != null) {
            kalmanFilter.DrawGizmos(
                kalmanParams.drawConfirmedLandmarks, 
                kalmanParams.drawPotentialLandmarks, 
                kalmanParams.drawObservations);
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

    // Return the real pose of the robot (for logging/debugging purposes only):
    public VehicleState GetRobotRealState() {
        return controller.GetRobotRealState();
    }

    public Lidar GetLidar() {
        return lidar;
    }
}