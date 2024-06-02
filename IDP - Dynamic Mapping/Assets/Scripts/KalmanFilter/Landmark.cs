using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class Landmark {
    public const int DIMENSION = 2;
    
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
        return Vector<float>.Build.DenseOfArray(new float[] { x, y });
    }

    public void setPosition(Vector<float> position) {
        this.x = position[0];
        this.y = position[1];
    }

    public override string ToString() {
        string print_x = Utils.ScientificNotation(x);
        string print_y = Utils.ScientificNotation(y);

        return "[x: " + print_x + ", y: " + print_y + "]";
    }
}