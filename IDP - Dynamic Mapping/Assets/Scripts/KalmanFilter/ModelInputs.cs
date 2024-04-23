public struct ModelInputs {
    public float V;
    public float gamma;

    public ModelInputs(float V, float gamma) {
        this.V = V;
        this.gamma = gamma;
    }

    public override string ToString() {
        return "[V: " + V + ", gamma: " + gamma + "]";
    }
}