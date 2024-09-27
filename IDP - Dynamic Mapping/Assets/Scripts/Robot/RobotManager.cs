using UnityEngine;
using UnityEngine.Profiling;

public class RobotManager : MonoBehaviour {

    public Robot robot;

    public bool drawLandmarkCandidates = true;

    // For now, there is no possibility to disable the mapping using grid maps, since
    // this mapping is more reliable to detect if a landmark is static or dynamic, and is
    // thus required for the localization estimate in a dynamic environment:
    public GridMapParams gridMapParams;

    public bool runGeometryClustering = true;
    public GeometryClusterParams geometryClusterParams;

    private GridMapBresenham worldGridMap;
    private GeometryMapping geometryClustering;

    private long updateCount = 0;

    void Start() {
        // Instantiate the grid map:
        worldGridMap = new GridMapBresenham(gridMapParams);

        // Instantiate the geometry clustering algorithm:
        if(runGeometryClustering)
            geometryClustering = new GeometryMapping(geometryClusterParams);
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

                if(geometryClustering != null) {
                    Profiler.BeginSample("Geometry clustering update");
                    geometryClustering.UpdateModel(
                        worldSensorPose, model,
                        data.vehicleState, data.vehicleStateCovariance,
                        data.observations[i], Time.time);
                    Profiler.EndSample();
                }
            }

            updateCount++;
        }
    }

    public void OnDrawGizmos() {
        if (worldGridMap != null) {
            worldGridMap.DrawMap(gridMapParams.showTexture,
                updateCount % gridMapParams.textureUpdateWait == 0,
                drawLandmarkCandidates);
        }

        if (geometryClustering != null) {
            geometryClustering.DrawGizmos();
        }

        // For testing, draw the match grid map dynamically when options in the editor are changed.
        // This is only executed when the application is not playing:
        if (!Application.isPlaying && geometryClusterParams.drawMatchGridMap) {
            GridMap.DrawGizmos(0.2f, 
                geometryClusterParams.width, geometryClusterParams.height,
                geometryClusterParams.centerX, geometryClusterParams.centerY, 
                geometryClusterParams.cellSize);
        }   
    }

    public WorldModel GetWorldModel() {
        return worldGridMap;
    }
}
