using UnityEngine;

public struct ModelInputs {
    public float V;
    public float gamma;

    public ModelInputs(float V, float gamma) {
        this.V = V;
        this.gamma = gamma;
    }

    public override string ToString() {
        string print_v = Utils.ScientificNotation(V);
        string print_g = Utils.ScientificNotation(Mathf.Rad2Deg * gamma);

        return "[V: " + print_v + ", gamma: " + print_g + "°]";
    }
}