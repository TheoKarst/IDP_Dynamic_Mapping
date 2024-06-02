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
        string print_r = Utils.ScientificNotation(r);
        string print_t = Utils.ScientificNotation(Mathf.Rad2Deg * theta);

        return "[r: " + print_r + ", theta: " + print_t + "°]";
    }
}