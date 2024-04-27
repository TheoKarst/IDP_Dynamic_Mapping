using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public struct Observation {
    public float r;
    public float theta;

    private static VectorBuilder<float> V = Vector<float>.Build;

    public Observation(float r, float theta) {
        this.r = r;
        this.theta = theta;
    }

    public static Vector<float> substract(Observation a, Observation b) {

        // The error on the angle should be between -pi and pi:
        float deltaAngle = a.theta - b.theta;
        while(deltaAngle > Mathf.PI) deltaAngle -= 2 * Mathf.PI;
        while(deltaAngle < -Mathf.PI) deltaAngle += 2 * Mathf.PI;

        return V.Dense(new float[] {
            a.r - b.r,
            deltaAngle
        });
    }

    public override string ToString() {
        float print_r = Mathf.Round(100 * r) / 100;
        float print_t = Mathf.Round(100 * theta * Mathf.Rad2Deg) / 100;

        return "[r: " + print_r + ", theta: " + print_t + "°]";
    }
}