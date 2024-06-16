﻿using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;
using static DataloaderRobot;

public class SimulatedRobot : MonoBehaviour {

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
    public float raycastDistance = 10;
    public float douglasPeuckerEpsilon;

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
        lidar = new Lidar(lidarObject, raycastCount, raycastDistance);

        // Instantiate the model we are going to use for the robot:
        (float a, float b) = lidar.GetLocalPosition();
        vehicleModel = new VehicleModel(a, b, controllerParams.L, controllerParams.maxSpeed,
            Mathf.Deg2Rad * controllerParams.maxSteering, waitBetweenUpdates);

        // Instantiate the script to move the robot with arrow keys:
        controller = new RobotController(robotObject, controllerParams);

        // Instantiate the Kalman Filter to estimate the robot position. We initialize
        // it with the real pose of the robot:
        robotStateEstimate = controller.GetRobotRealState();
        Logger kalmanLogger = kalmanParams.writeLogFile ? new Logger(false, "kalman_estimate.csv") : new Logger(false);
        kalmanFilter = new KalmanFilter(this, robotStateEstimate, vehicleModel, kalmanParams, kalmanLogger);
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
            AugmentedObservation[] observations = lidar.ComputeObservations();

            // Get the current inputs of the robot:
            ModelInputs inputs = controller.GetModelInputs();

            // Get the observations from the LIDAR that are good landmarks candidates:
            int[] filteredObservations = LidarUtils.DouglasPeucker(observations, douglasPeuckerEpsilon);
            List<int> landmarkCandidates = LidarUtils.ExtractConvexCorners(observations, filteredObservations, 220);

            // Use the static landmark candidates to update our state estimate. To find the static
            // landmarks, we need a state estimate. For that we can use the vehicle model and the
            // previous state estimate. Using these landmarks, the Kalman Filter will then provide
            // a better estimate of the real state of the robot:
            VehicleState statePrediction = vehicleModel.PredictCurrentState(robotStateEstimate, inputs, currentTime - lastTimeUpdate);
            List<Observation> staticLandmarkCandidates = LidarUtils.GetStaticObservations(observations, landmarkCandidates, manager.GetWorldModel(), vehicleModel, statePrediction);

            Debug.Log("Landmark candidates: " + landmarkCandidates.Count + " (" + staticLandmarkCandidates.Count + " are static)");
            // Use these observations to update the robot state estimate:
            kalmanFilter.UpdateStateEstimate(staticLandmarkCandidates, inputs, currentTime);
            VehicleState vehicleState = kalmanFilter.GetStateEstimate();
            Matrix<double> vehicleStateCovariance = kalmanFilter.GetStateCovarianceEstimate();

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

    // Return the real pose of the robot (for logging/debugging purposes only):
    public VehicleState GetRobotRealState() {
        return controller.GetRobotRealState();
    }

    public Lidar GetLidar() {
        return lidar;
    }
}