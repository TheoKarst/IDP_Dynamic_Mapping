using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.UI;

public class RobotManager : MonoBehaviour {

    [Serializable]
    public class ControllerParams {
        public bool forceForward = false;
        public bool forceBackward = false;
        public bool forceLeft = false;
        public bool forceRight = false;

        public float acceleration = 10;
        public float L = 0.3f;

        public float maxSpeed = 2f;
        public float maxSteering = 10;

        public float friction = 10;
    }

    [Serializable]
    public class LidarParams {
        public int raycastCount = 500;
        public float raycastDistance = 10;
        public float douglasPeuckerEpsilon = 0.2f;

        public bool drawRays = true;
        public bool drawCorners = true;
    }

    [Serializable]
    public class GridMapParams {
        public RawImage mapImage;

        // Number of updates to wait before updating the texture:
        public int textureUpdateWait = 10;
        public bool showTexture = false;

        public int pixelSize = 1;
        public float cellSize = 0.1f;
        public int mapWidth = 100;
        public int mapHeight = 100;

        public float maxConfidence = 0.99f;

        [Tooltip("Average number of frames to wait before considering an object as static in the static grid")]
        public int framesToConsiderStatic = 60;

        [Tooltip("Average number of frames to wait before considering an object as dynamic in the dynamic grid")]
        public int framesToConsiderDynamic = 20;
    }

    [Serializable]
    public class KalmanParams {
        public bool writeLogFile = false;

        [Tooltip("The maximum number of landmarks we can use to update the robot " +
            "state estimate (more landmarks means more precision, but is also slower)")]
        public int maxLandmarksPerUpdate = 5;

        [Tooltip("Minimum distance between landmarks (avoid having too many landmarks at the same place)")]
        public float minDistanceBetweenLandmarks = 1;

        public bool drawConfirmedLandmarks = false;
        public bool drawPotentialLandmarks = false;
        public bool drawObservations = false;
    }

    [Serializable]
    public class GeometryClusterParams {
        [Header("Geometry Extraction:")]
        [Tooltip("Maximum distance between two consecutive points to be matched to the same line")]
        public float PointCriticalDistance = 0.2f;

        [Tooltip("Maximum orthogonal distance of a point to a matching line")]
        public float LineCriticalDistance = 0.06f;

        [Tooltip("Maximum angle (in degrees) between two consecutive points to be matched to the same line")]
        public float CriticalAlpha = 1f;

        [Tooltip("Maximum distance between two consecutive points to be matched to the same circle cluster")]
        public float CircleCriticalDistance = 0.2f;

        [Tooltip("Minimum length of a line (smaller lines are considered as circle clusters)")]
        public float LineMinLength = 0.3f;

        [Header("Geometry Matching:")]
        [Tooltip("Maximum angle (in degrees) between two lines to be matched together")]
        public float LineMaxMatchAngle = 10f;

        [Tooltip("Maximum distance between the endpoint of two lines to be matched together")]
        public float LineMaxEndpointMatchDistance = 0.1f;

        [Tooltip("Maximum distance between two lines to be matched together")]
        public float LineMaxMatchDistance = 0.2f;

        [Tooltip("Maximum distance between two circles to be matched together")]
        public float CircleMaxMatchDistance = 0.2f;

        [Header("Geometry removing")]
        [Tooltip("Extent of the wipe triangle built for new lines")]
        public float WipeTriangleExtent = 0.1f;

        public float WipeShapeExtent = 0.1f;

        [Tooltip("Margin angle (in degrees) of the wipe triangles")]
        public float WipeTriangleInsideAngleMargin = 1f;

        [Tooltip("Minimum distance between a circle and a line in the model")]
        public float MinCircleLineDistance = 0.5f;

        [Header("Drawing")]
        public bool drawPoints = true;
        public bool drawLines = true;
        public bool drawCircles = true;
        public bool drawCurrentLines = true;
        public bool drawWipeShape = true;
    }

    [Header("General")]
    public GameObject robotObject;
    public GameObject lidarObject;

    [Tooltip("Wait before performing a new lidar scan, position estimate and map update")]
    public float waitBetweenUpdates = 0.02f;

    [Header("Robot")]
    public ControllerParams controllerParams;
    public LidarParams lidarParams;

    [Header("Kalman Filter")]
    public KalmanParams kalmanParams;

    [Header("Grid maps")]
    public GridMapParams gridMapParams;

    [Header("Geometry Clustering")]
    public GeometryClusterParams geometryClusterParams;

    private Lidar lidar;
    private VehicleModel vehicleModel;
    private RobotController controller;
    private KalmanFilter kalmanFilter;
    private GridMapBresenham worldGridMap;
    private GeometryClustering geometryClustering;

    // Last time we performed a complete update (lidar measures + robot state estimate + map update):
    private float lastTimeUpdate = 0;
    private long updateCount = 0;

    void Start() {
        // Instantiate the LIDAR:
        lidar = new Lidar(lidarObject, lidarParams);

        // Instantiate the grid map:
        worldGridMap = new GridMapBresenham(this, gridMapParams);
        
        // Instantiate the model we are going to use for the robot:
        vehicleModel = new VehicleModel(lidar, controllerParams.L, controllerParams.maxSpeed, 
            Mathf.Deg2Rad * controllerParams.maxSteering, waitBetweenUpdates);

        // Instantiate the script to move the robot with arrow keys:
        controller = new RobotController(robotObject, controllerParams);

        // Instantiate the Kalman Filter to estimate the robot position:
        VehicleState initialState = controller.GetRobotRealState();
        Logger kalmanLogger = kalmanParams.writeLogFile ? new Logger(false, "kalman_estimate.csv") : new Logger(false);
        kalmanFilter = new KalmanFilter(this, initialState, vehicleModel, kalmanParams, kalmanLogger);

        // Instantiate the geometry clustering algorithm:
        geometryClustering = new GeometryClustering(geometryClusterParams, lidarParams.raycastCount);
    }

    // Update is called once per frame
    void Update() {
        // Toggle the grid map display when pressing "g":
        if(Input.GetKeyDown(KeyCode.G))
            gridMapParams.showTexture = !gridMapParams.showTexture;

        // Each frame, update the robot position:
        controller.Update();

        // Perform a whole update less often (in reality, the robot state in continuous, while the LIDAR
        // measures, the state estimate and map update are discrete):
        if(Time.time - lastTimeUpdate >= waitBetweenUpdates) {
            // Use raycasting to compute observations from the LIDAR, detect corners among 
            // these observations, etc...
            lidar.Update();
            
            // Get the observations from the LIDAR that are good landmarks candidates:
            List<Observation> landmarkCandidates = lidar.GetLandmarkCandidates(worldGridMap);

            // Get the current inputs of the robot:
            ModelInputs inputs = controller.GetModelInputs();

            // Use these observations to update the robot state estimate:
            Profiler.BeginSample("Kalman Update");
            kalmanFilter.UpdateStateEstimate(landmarkCandidates, inputs, Time.time);
            Profiler.EndSample();

            VehicleState vehicleState = kalmanFilter.GetStateEstimate();
            Matrix<double> vehicleStateCovariance = kalmanFilter.GetStateCovarianceEstimate();

            // From the vehicle model, and knowing the state of the vehicle, compute the position
            // of the sensor (the LIDAR):
            Vector2 sensorPosition = vehicleModel.GetSensorPosition(vehicleState);
            float sensorAngle = vehicleState.phi;

            // Build the WipeShape, using the vehicle state:
            Profiler.BeginSample("Build Wipe Shape");
            WipeShape wipeShape = lidar.BuildWipeShape(this, vehicleState, sensorPosition, geometryClusterParams.WipeShapeExtent);
            Profiler.EndSample();

            // Get the observations from the LIDAR, and use them to update the world model:
            ExtendedObservation[] observations = lidar.GetExtendedObservations();

            // Finally update the grid maps:
            Profiler.BeginSample("Grid Map Update");
            worldGridMap.UpdateMaps(sensorPosition, sensorAngle, observations);
            Profiler.EndSample();

            // And update the geometry clustering algorithm:
            Profiler.BeginSample("Geometry Clustering");
            geometryClustering.UpdateModel(this, vehicleState, vehicleStateCovariance, observations, wipeShape);
            Profiler.EndSample();

            lastTimeUpdate = Time.time;
            updateCount++;
        }
    }

    public void OnDrawGizmos() {
        if(lidar != null)
            lidar.DrawGizmos(lidarParams.drawRays, lidarParams.drawCorners);

        if (kalmanFilter != null) {
            kalmanFilter.DrawGizmos(kalmanParams.drawConfirmedLandmarks,
                kalmanParams.drawPotentialLandmarks,
                kalmanParams.drawObservations);
        }

        if (worldGridMap != null) {
            worldGridMap.DrawMap(gridMapParams.showTexture,
                updateCount % gridMapParams.textureUpdateWait == 0);
        }

        if(geometryClustering != null) {
            geometryClustering.DrawGizmos(
                geometryClusterParams.drawCurrentLines, 
                geometryClusterParams.drawPoints, 
                geometryClusterParams.drawLines, 
                geometryClusterParams.drawCircles, 
                geometryClusterParams.drawWipeShape);
        }
    }

    // Return the real pose of the robot (for logging/debugging purposes only):
    public VehicleState GetRobotRealState() {
        return controller.GetRobotRealState();
    }

    public Lidar GetLidar() {
        return lidar;
    }

    public VehicleModel GetVehicleModel() {
        return vehicleModel;
    }

    public Vector<double> ComputeObservationPositionEstimate(Observation observation) {
        VehicleState stateEstimate = kalmanFilter.GetStateEstimate();
        return vehicleModel.ComputeObservationPositionEstimate(stateEstimate, observation);
    }
}
