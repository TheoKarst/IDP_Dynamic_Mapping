using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public struct VehicleState {
    public const int DIMENSION = 3;

    public float x;
    public float y;
    public float phi;

    public VehicleState(float x, float y, float phi) {
        this.x = x;
        this.y = y;
        this.phi = phi;
    }

    public static VehicleState operator +(VehicleState state, Vector<float> other) {
        return new VehicleState(state.x + other[0], state.y + other[1], state.phi + other[2]);
    }

    public static VehicleState operator -(VehicleState state, Vector<float> other) {
        return new VehicleState(state.x - other[0], state.y - other[1], state.phi - other[2]);
    }

    public Vector<float> ToVector() {
        return Vector<float>.Build.Dense(new float[] { x, y, phi });
    }

    public override string ToString() {
        string print_x = Utils.ScientificNotation(x);
        string print_y = Utils.ScientificNotation(y);
        string print_phi = Utils.ScientificNotation(Mathf.Rad2Deg * phi);

        return "[x: " + print_x + ", y: " + print_y + ", phi: " + print_phi + "°]"; 
    }
}