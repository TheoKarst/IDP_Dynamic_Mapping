using MathNet.Numerics.LinearAlgebra;

public class Landmark {
    public const int DIMENSION = 2;
    
    public float x;
    public float y;

    public Landmark(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public Landmark(Vector<double> position) {
        setPosition(position);
    }

    public Vector<double> getPosition() {
        return Vector<double>.Build.DenseOfArray(new double[] { x, y });
    }

    public void setPosition(Vector<double> position) {
        this.x = (float) position[0];
        this.y = (float) position[1];
    }

    public override string ToString() {
        string print_x = Utils.ScientificNotation(x);
        string print_y = Utils.ScientificNotation(y);

        return "[x: " + print_x + ", y: " + print_y + "]";
    }
}