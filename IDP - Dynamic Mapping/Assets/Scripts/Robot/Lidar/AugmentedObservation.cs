// Class used to represent an observation from the LIDAR, plus some additional
// informations:
public class AugmentedObservation {
    public float r;
    public float theta;
    public int lidarIndex;

    // If the observation is out of the max range of the LIDAR (in this case, the
    // observation radius is cropped to the maximum range):
    public bool outOfRange;

    public AugmentedObservation(float r, float theta, int lidarIndex, bool outOfRange) {
        this.r = r;
        this.theta = theta;
        this.lidarIndex = lidarIndex;
        this.outOfRange = outOfRange;
    }

    public Observation ToObservation() {
        return new Observation(r, theta, lidarIndex);
    }
}
