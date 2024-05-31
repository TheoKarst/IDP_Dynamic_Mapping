// A class to represent an observation with some more infos:
using UnityEngine;

public struct ExtendedObservation {
    public float r;
    public float theta;
    public int index;           // Index of the observation in the list of observations of the LIDAR
    public bool isValid;        // If the observation corresponds to a hit point, or just an "out of range"

    public ExtendedObservation(float r, float theta, int index, bool isValid) {
        this.r = r;
        this.theta = theta;
        this.index = index;
        this.isValid = isValid;
    }

    public Observation ToObservation() {
        if (!isValid)
            Debug.LogError("Trying to create an invalid observation");

        return new Observation(r, theta);
    }
}
