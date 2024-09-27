using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

/// <summary>
/// Struct used to represent the observations made by a LIDAR
/// </summary>

public struct Observation {
    public const int DIMENSION = 2;

    public float r;                 // Range of the observation
    public float theta;             // Angle of the observation (in radians)

    public int lidarIndex;          // Index of the LIDAR which made the observation
    public bool outOfRange;         // If the observation was out of the range of the LIDAR

    public Observation(float r, float theta, int lidarIndex, bool outOfRange) {
        this.r = r;
        this.theta = theta;
        this.lidarIndex = lidarIndex;
        this.outOfRange = outOfRange;
    }

    public static void Substract(Observation a, Observation b, Vector<double> dest, int index) {
        dest[index] = a.r - b.r;
        dest[index + 1] = Utils.SubstractAngleRadians(a.theta, b.theta);
    }

    public override string ToString() {
        string print_r = Utils.ToString(r);
        string print_t = Utils.ToString(Mathf.Rad2Deg * theta);

        return "[Lidar n°" + lidarIndex + ": r: " + print_r 
            + ", theta: " + print_t + "°, out of range: " + outOfRange + "]";
    }
}