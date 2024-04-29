using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class Landmark {
    private static VectorBuilder<float> V = Vector<float>.Build;

    public float x;
    public float y;

    public Landmark(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Landmark(Vector<float> position) {
        setPosition(position);
    }

    public Vector<float> getPosition() {
        return V.DenseOfArray(new float[] { x, y });
    }

    public void setPosition(Vector<float> position) {
        this.x = position[0];
        this.y = position[1];
    }

    public override string ToString() {
        float print_x = Mathf.Round(100 * x) / 100;
        float print_y = Mathf.Round(100 * y) / 100;

        return "[x: " + print_x + ", y: " + print_y + "]";
    }
}