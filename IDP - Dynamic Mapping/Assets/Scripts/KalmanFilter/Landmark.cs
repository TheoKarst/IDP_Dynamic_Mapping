using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;

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
        return "[x: " + x + ", y: " + y + "]";
    }
}