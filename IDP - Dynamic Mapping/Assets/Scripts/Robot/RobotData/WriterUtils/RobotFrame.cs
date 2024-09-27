
/// <summary>
/// Serializable class used to represent a frame of data (contains the frame number, the timestamp,
/// the pose of the robot and the data from the LIDARs)
/// </summary>

[System.Serializable]
public class RobotFrame {
    // Number of the current frame (starting from zero):
    public int frame_number;

    // Timestamp (in seconds) of the current frame (the first frame has a timestamp of zero):
    public float timestamp;

    // Current pose of the robot:
    public Pose2D robot_pose;

    // Data of every LIDAR registered in the RobotSetup config file:
    public LidarData[] lidars_data;

    public RobotFrame(int frame_number, float timestamp, Pose2D robot_pose, LidarData[] lidars_data) {
        this.frame_number = frame_number;
        this.timestamp = timestamp;
        this.robot_pose = robot_pose;
        this.lidars_data = lidars_data;
    }
}
