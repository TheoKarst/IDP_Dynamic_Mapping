using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Class used to represent the landmarks used for localization
/// </summary>

public class Landmark {
    public const int DIMENSION = 2;
    
    public float x;
    public float y;

    public Landmark(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Landmark(Vector<double> position) {
        SetPosition(position);
    }

    public Vector<double> GetPosition() {
        return Vector<double>.Build.DenseOfArray(new double[] { x, y });
    }

    public void SetPosition(Vector<double> position) {
        this.x = (float) position[0];
        this.y = (float) position[1];
    }

    public override string ToString() {
        string print_x = Utils.ToString(x);
        string print_y = Utils.ToString(y);

        return "[x: " + print_x + ", y: " + print_y + "]";
    }
}