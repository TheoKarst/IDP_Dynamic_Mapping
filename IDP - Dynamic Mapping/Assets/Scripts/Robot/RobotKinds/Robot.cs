using UnityEngine;

/// <summary>
/// Class used to represent a robot, able to produce LIDAR data that can be used
/// to build a map of the environment. There are currently 4 different kinds of robots:
/// - DataloaderRobot: Used to play recorded LIDAR captures
/// - KalmanRobot: Used to represent a real robot (with the localization estimate from the Kalman Filter)
/// - RobotRecorder: Used to record LIDAR captures
/// - SimulatedRobot: Used to represent a simulated robot (with the localization given by Unity)
/// </summary>

public abstract class Robot : MonoBehaviour {

    /// <summary>
    /// Returns if a new frame of data is currently available. If this function returns true,
    /// the frame of data is made available by the function GetCurrentFrame()
    /// </summary>
    public abstract bool IsNewFrameAvailable();

    /// <summary>
    /// Returns the last frame of data that was produced by the robot
    /// </summary>
    public abstract RobotData GetCurrentFrame();

    /// <summary>
    /// Returns the model representing the robot
    /// </summary>
    public abstract VehicleModel GetVehicleModel();
}
