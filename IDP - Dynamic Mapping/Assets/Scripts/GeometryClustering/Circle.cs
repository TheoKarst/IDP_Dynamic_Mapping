using UnityEngine;

public class Circle {
    // Center and radius of the circle:
    private float xc, yc, R;

    public Vector2 position { get => new Vector2(xc, yc); }

    public Circle(float xc, float yc, float R) {
        this.xc = xc;
        this.yc = yc;
        this.R = R;
    }

    public void DrawGizmos() {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(new Vector3(xc, 0.5f, yc), R);
    }

    // Supposing that this circle belongs to the current model of the environment, use the
    // given circle (that is supposed to be matched with this one) to update the center and
    // radius of this circle:
    public void UpdateCircleUsingMatching(Circle other) {
        xc = (xc + other.xc) / 2;
        yc = (yc + other.yc) / 2;
        R = (R + other.R) / 2;
    }

    // Compute the Euclidean distance between the two circles centers:
    public float DistanceFrom(Circle other) {
        float dX = other.xc - xc;
        float dY = other.yc - yc;

        return Mathf.Sqrt(dX * dX + dY * dY);
    }
}
