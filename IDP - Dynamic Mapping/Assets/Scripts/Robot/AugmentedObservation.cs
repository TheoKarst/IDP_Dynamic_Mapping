// Class used to represent an observation from the LIDAR, plus some additional
// informations:
public class AugmentedObservation {
    public float r;
    public float theta;

    // If the observation is out of the max range of the LIDAR (in this case, the
    // observation radius is cropped to the maximum range):
    public bool outOfRange;

    public AugmentedObservation(float r, float theta, bool outOfRange) {
        this.r = r;
        this.theta = theta;
        this.outOfRange = outOfRange;
    }
}
