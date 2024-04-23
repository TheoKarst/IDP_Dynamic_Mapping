public struct Observation {
    public float r;
    public float theta;

    public Observation(float r, float theta) {
        this.r = r;
        this.theta = theta;
    }

    public override string ToString() {
        return "[r: " + r + ", theta: " + theta + "]";
    }
}