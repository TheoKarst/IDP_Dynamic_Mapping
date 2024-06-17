using UnityEngine;

public class RobotManager : MonoBehaviour {

    public Robot robot;

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

            // Get the vehicle model of the robot to compute the pose of each LIDAR on the robot:
            VehicleModel model = robot.GetVehicleModel();

            // Then, for each LIDAR in the current frame, use the LIDAR observations to update the world models:
            for(int i = 0; i < data.observations.Length; i++) {
                Pose2D sensorPose = model.GetWorldSensorPose(data.vehicleState, i);

                worldGridMap.UpdateMaps(sensorPose, data.observations[i]);

                geometryClustering.UpdateModel(
                    sensorPose, model,
                    data.vehicleState, data.vehicleStateCovariance,
                    data.observations[i]);
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
