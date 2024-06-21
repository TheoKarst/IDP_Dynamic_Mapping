
using UnityEngine;

public abstract class Robot : MonoBehaviour {
    public abstract bool IsNewFrameAvailable();

    public abstract DataloaderRobot.RobotData GetCurrentFrame();

    public abstract VehicleModel GetVehicleModel();
}
