[System.Serializable]
public class LidarData {
    // Id of the LIDAR sending this data:
    public int id;

    // Angles (in radians) corresponding to the observations of the LIDAR. The angles are
    // defined in counter-clockwise order, starting from the front of the LIDAR:
    public float[] angles;

    // Range (in meters) of all the observations, corresponding to the previous angles:
    public float[] ranges;

    public LidarData(int id, float[] angles, float[] ranges) {
        this.id = id;
        this.angles = angles;
        this.ranges = ranges;
    }
}