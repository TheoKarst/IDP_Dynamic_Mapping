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

    public static VehicleState operator +(VehicleState state, Vector<double> other) {
        return new VehicleState(
            state.x + (float) other[0], 
            state.y + (float) other[1], 
            state.phi + (float) other[2]);
    }

    public static VehicleState operator -(VehicleState state, Vector<double> other) {
        return new VehicleState(
            state.x - (float) other[0], 
            state.y - (float) other[1], 
            state.phi - (float) other[2]);
    }

    public Pose2D GetPose() {
        return new Pose2D(x, y, phi);
    }

    public override string ToString() {
        string print_x = Utils.ToString(x);
        string print_y = Utils.ToString(y);
        string print_phi = Utils.ToString(Mathf.Rad2Deg * phi);

        return "[x: " + print_x + ", y: " + print_y + ", phi: " + print_phi + "°]"; 
    }
}