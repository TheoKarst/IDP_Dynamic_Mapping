[System.Serializable]
public class RobotSetup {
    // Initial global pose of the robot in the scene:
    public Pose2D initial_robot_pose;

    // Initial setup of all the LIDARs on the robot:
    public LidarSetup[] lidars_setup;

    public RobotSetup(Pose2D initial_robot_pose, LidarSetup[] lidars_setup) {
        this.initial_robot_pose = initial_robot_pose;
        this.lidars_setup = lidars_setup;
    }
}