using UnityEngine;

public class RobotManager : MonoBehaviour {

    public SimulatedRobot robot;

    public GridMapParams gridMapParams;
    public GeometryClusterParams geometryClusterParams;

    private GridMapBresenham worldGridMap;
    private GeometryClustering geometryClustering;

    private long updateCount = 0;

    void Start() {
        // Instantiate the grid map:
        worldGridMap = new GridMapBresenham(gridMapParams);
        
        // Instantiate the geometry clustering algorithm:
        geometryClustering = new GeometryClustering(geometryClusterParams);
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

            // Also get the current pose of the LIDAR:
            Pose2D sensorPose = robot.GetSensorPose();

            // Then update the grid maps:
            worldGridMap.UpdateMaps(sensorPose, data.observations);

            // And update the geometry clustering algorithm:
            geometryClustering.UpdateModel(
                sensorPose, robot.GetVehicleModel(), 
                data.vehicleState, data.vehicleStateCovariance, 
                data.observations);

            updateCount++;
        }
    }

    public void OnDrawGizmos() {
        if (worldGridMap != null) {
            worldGridMap.DrawMap(gridMapParams.showTexture,
                updateCount % gridMapParams.textureUpdateWait == 0);
        }

        if (geometryClustering != null) {
            geometryClustering.DrawGizmos(
                geometryClusterParams.drawCurrentLines, 
                geometryClusterParams.drawPoints, 
                geometryClusterParams.drawLines, 
                geometryClusterParams.drawCircles, 
                geometryClusterParams.drawWipeShape);
        }
    }

    public WorldModel GetWorldModel() {
        return worldGridMap;
    }
}
