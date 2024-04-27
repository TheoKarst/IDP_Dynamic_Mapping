using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public struct ModelState {
    public float x;
    public float y;
    public float phi;

    public ModelState(float x, float y, float phi) {
        this.x = x;
        this.y = y;
        this.phi = phi;
    }

    public static ModelState operator +(ModelState state, Vector<float> other) {
        return new ModelState(state.x + other[0], state.y + other[1], state.phi + other[2]);
    }

    public static ModelState operator -(ModelState state, Vector<float> other) {
        return new ModelState(state.x - other[0], state.y - other[1], state.phi - other[2]);
    }

    public ModelState copy() {
        return new ModelState(x, y, phi);
    }

    public override string ToString() {
        float print_x = Mathf.Round(100 * x) / 100;
        float print_y = Mathf.Round(100 * y) / 100;
        float print_phi = Mathf.Round(100 * phi * Mathf.Rad2Deg) / 100;

        return "[x: " + print_x + ", y: " + print_y + ", phi: " + print_phi + "°]"; 
    }
}