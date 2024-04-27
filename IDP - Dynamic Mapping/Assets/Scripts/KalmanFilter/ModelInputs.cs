using UnityEngine;

public struct ModelInputs {
    public float V;
    public float gamma;

    public ModelInputs(float V, float gamma) {
        this.V = V;
        this.gamma = gamma;
    }

    public override string ToString() {
        float print_v = Mathf.Round(100 * V) / 100;
        float print_g = Mathf.Round(100 * gamma * Mathf.Rad2Deg) / 100;

        return "[V: " + print_v + ", gamma: " + print_g + "°]";
    }
}