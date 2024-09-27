using UnityEngine;

/// <summary>
/// Struct used to represent the inputs of the robot (speed and steering angle)
/// </summary>

public struct ModelInputs {
    public float V;             // Linear speed of the robot
    public float gamma;         // Steering angle of the front wheels of the robot

    public ModelInputs(float V, float gamma) {
        this.V = V;
        this.gamma = gamma;
    }

    public override string ToString() {
        string print_v = Utils.ToString(V);
        string print_g = Utils.ToString(Mathf.Rad2Deg * gamma);

        return "[V: " + print_v + ", gamma: " + print_g + "°]";
    }
}