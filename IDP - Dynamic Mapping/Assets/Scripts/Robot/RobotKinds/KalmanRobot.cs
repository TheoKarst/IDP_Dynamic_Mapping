﻿using System.Collections.Generic;
using UnityEngine;
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

    [Tooltip("Number of ")]
    public int raycastCount = 500;
    public float lidarMinRange = 0;
    public float lidarMaxRange = 10;
    public float douglasPeuckerEpsilon = 0.2f;

    public bool drawRays = true;

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

    void Start() {
        // Instantiate the LIDAR:
        lidar = new Lidar(lidarObject, raycastCount, lidarMinRange, lidarMaxRange, 0);

        // Instantiate the model we are going to use for the robot:
        LidarSetup[] lidarSetups = new LidarSetup[] { lidar.GetSetup() };
        vehicleModel = new VehicleModel(lidarSetups, controllerParams.L);

        // Instantiate the script to move the robot with arrow keys:
        controller = new RobotController(robotObject, controllerParams);

        // Instantiate the Kalman Filter to estimate the robot position. We initialize
        // it with the real pose of the robot:
        VehicleState initialState = controller.GetRobotRealState();
        Logger kalmanLogger = kalmanParams.writeLogFile ? new Logger(false, "./Assets/Data/logs/kalman_estimate.csv") : new Logger(false);
        kalmanFilter = new KalmanFilter(this, initialState, vehicleModel, kalmanParams, kalmanLogger);
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
            Observation[] observations = lidar.ComputeObservations();

            // Get the current inputs of the robot:
            ModelInputs inputs = controller.GetModelInputs();

            // Get the observations from the LIDAR that are good landmarks candidates:
            Observation[] filteredObservations = Filtering.DouglasPeucker(observations, douglasPeuckerEpsilon);
            List<Observation> landmarkCandidates = Filtering.ExtractConcaveCorners(filteredObservations, 220);

            // Using the world model, we want to identify which landmarks corresponds to static objects. For that, we need
            // a pose estimate for the robot. Since we still haven't identified which landmarks to use for localisation,
            // we have to rely only on the prediction step for the robot state estimate:
            GridMapBresenham worldModel = manager.GetWorldModel();

            // If we have a world model, we can remove the dynamic observations from the landmark candidates:
            if(worldModel != null) {
                // Get the previous state estimate:
                VehicleState previousStateEstimate = kalmanFilter.GetStateEstimate();

                // Perform the prediction step of the Kalman Filter to estimate the current pose of the robot.
                // We cannot use the update step of the Kalman Filter here, since we are still trying to figure out
                // which observations to use for the state correction:
                VehicleState statePrediction = vehicleModel.PredictCurrentState(previousStateEstimate, inputs, currentTime - lastTimeUpdate);

                // From this estimate, we can compute the world space position of the observations and detect if they are static:
                landmarkCandidates = Filtering.GetStaticObservations(landmarkCandidates, worldModel, vehicleModel, statePrediction);
            }

            // Use these observations to update the robot state estimate:
            kalmanFilter.UpdateStateEstimate(landmarkCandidates, inputs, currentTime);

            // Create a new data frame to represent the current state of the robot:
            currentFrame = new RobotData(currentTime, 
                kalmanFilter.GetStateEstimate(), 
                kalmanFilter.GetStateCovarianceEstimate(), 
                new Observation[][] { observations });

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