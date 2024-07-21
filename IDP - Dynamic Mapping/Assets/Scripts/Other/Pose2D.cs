[System.Serializable]
public struct Pose2D {
    public float x;
    public float y;
    public float angle;

    public Pose2D(float x, float y, float angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
}