using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public struct Landmark {
    private static VectorBuilder<float> V = Vector<float>.Build;

    public float x;
    public float y;

    public Landmark(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Vector<float> position() {
        return V.DenseOfArray(new float[] { x, y });
    }

    public override string ToString() {
        float print_x = Mathf.Round(100 * x) / 100;
        float print_y = Mathf.Round(100 * y) / 100;

        return "[x: " + print_x + ", y: " + print_y + "]";
    }
}