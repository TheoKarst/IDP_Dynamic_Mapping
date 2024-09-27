/// <summary>
/// Serializable class used to represent the setup of a LIDAR. This contains
/// information like the local pose of the LIDAR on the robot, the id of the LIDAR,
/// the minimum and maximum range of the LIDAR
/// </summary>

[System.Serializable]
public class LidarSetup {
    // Id of the LIDAR: for each robot frame (see RobotFrame class), the data of this LIDAR
    // will be saved at lidars_data[id]:
    public int id;

    // Optional name of the LIDAR:
    public string name;

    // Local pose of the LIDAR on the robot:
    public Pose2D local_pose;

    // Min range of the LIDAR (values below this threshold should be discarded):
    public float min_range;

    // Max range of the LIDAR (values above this threshold should be clamped):
    public float max_range;

    public LidarSetup(int id, string name, Pose2D local_pose, float min_range, float max_range) {
        this.id = id;
        this.name = name;
        this.local_pose = local_pose;
        this.min_range = min_range;
        this.max_range = max_range;
    }
}