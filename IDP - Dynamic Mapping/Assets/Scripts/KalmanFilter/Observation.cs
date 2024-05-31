using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public struct Observation {
    public const int DIMENSION = 2;

    public float r;
    public float theta;

    public Observation(float r, float theta) {
        this.r = r;
        this.theta = theta;
    }

    public static void Substract(Observation a, Observation b, Vector<float> dest, int index) {
        dest[index] = a.r - b.r;
        dest[index + 1] = Utils.SubstractAngleRadians(a.theta, b.theta);
    }

    public override string ToString() {
        float print_r = Utils.Round(r, 2);
        float print_t = Utils.Round(Mathf.Rad2Deg * theta, 2);

        return "[r: " + print_r + ", theta: " + print_t + "°]";
    }
}