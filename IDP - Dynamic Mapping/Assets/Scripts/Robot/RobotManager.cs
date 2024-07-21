using UnityEngine;
using UnityEngine.Profiling;

public class RobotManager : MonoBehaviour {

    public Robot robot;

    [Header("Line match grid map")]
    public bool showMatchGridMap = false;
    public float centerX = 0;
    public float centerY = 0;
    public int width = 40;
    public int height = 40;
    public float cellSize = 2;

    public GridMapParams gridMapParams;
    public GeometryClusterParams geometryClusterParams;

    private GridMapBresenham worldGridMap;
    private GeometryClustering geometryClustering;
    private GridMap gridMap;

    private long updateCount = 0;

    void Start() {
        // Instantiate the grid map:
        worldGridMap = new GridMapBresenham(gridMapParams);
        
        // Instantiate the geometry clustering algorithm:
        geometryClustering = new GeometryClustering(geometryClusterParams, gridMap);
    }

    // Update is called once per frame
    void Update() {
        // Toggle the grid map display when pressing "g":
        if(Input.GetKeyDown(KeyCode.G))
            gridMapParams.showTexture = !gridMapParams.showTexture;

        // Update the robot state here (since it's done by Unity, we don't need to do so):
        // ...

        // If a new frame of data is available, use it to update the maps:
        if(robot.IsNewFrameAvailable()) {
            // Get the current data of the robot:
            DataloaderRobot.RobotData data = robot.GetCurrentFrame();

            // Get the vehicle model of the robot to compute the pose of each LIDAR on the robot:
            VehicleModel model = robot.GetVehicleModel();

            // Then, for each LIDAR in the current frame, use the LIDAR observations to update the world models:
            for(int i = 0; i < data.observations.Length; i++) {
                Pose2D worldSensorPose = model.GetWorldSensorPose(data.vehicleState, i);

                Profiler.BeginSample("Static/Dynamic maps update");
                worldGridMap.UpdateMaps(worldSensorPose, data.observations[i]);
                Profiler.EndSample();

                Profiler.BeginSample("Geometry clustering update");
                geometryClustering.UpdateModel(
                    worldSensorPose, model,
                    data.vehicleState, data.vehicleStateCovariance,
                    data.observations[i], Time.time);
                Profiler.EndSample();
            }

            updateCount++;
        }
    }

    public void OnDrawGizmos() {
        if (worldGridMap != null) {
            worldGridMap.DrawMap(gridMapParams.showTexture,
                updateCount % gridMapParams.textureUpdateWait == 0);
        }

        if (geometryClustering != null) {
            geometryClustering.DrawGizmos();
        }

        if (showMatchGridMap && gridMap != null)
            gridMap.DrawGizmos(0.2f);
    }

    public void OnValidate() {
        gridMap = new GridMap(width, height, centerX, centerY, cellSize);
    }

    public WorldModel GetWorldModel() {
        return worldGridMap;
    }
}
