using UnityEngine;

public class Circle : Primitive {
    public Color circleColor = Color.red;

    // Center and radius of the circle:
    private float xc, yc, R;
    private float xcP, ycP;     // Derivative of xc and yc (used to estimate the speed of the circle)

    public Vector2 position { get => new Vector2(xc, yc); }

    public Circle(float xc, float yc, float R) {
        this.xc = xc;
        this.yc = yc;
        this.R = R;

        this.xcP = this.ycP = 0;
    }

    public void DrawGizmos() {
        Gizmos.color = circleColor;
        Gizmos.DrawSphere(new Vector3(xc, 0.2f, yc), R);
    }

    // Supposing that this circle belongs to the current model of the environment, use the
    // given circle (that is supposed to be matched with this one) to update the center and
    // radius of this circle:
    public void UpdateCircleUsingMatching(Circle other) {
        float newXc = (xc + other.xc) / 2;
        float newYc = (yc + other.yc) / 2;
        R = (R + other.R) / 2;

        // Use the newXc and newYc to update the circle speed estimate, using a simple
        // exponential low pass filter:
        const float m = 0.9f;
        other.xcP = xcP = m * xcP + (1 - m) * (newXc - xc);
        other.ycP = ycP = m * ycP + (1 - m) * (newYc - yc);

        // Update xc and yc:
        xc = newXc; yc = newYc;
    }

    // Compute the Euclidean distance between the two circles centers:
    public float DistanceFrom(Circle other) {
        float dX = other.xc - xc;
        float dY = other.yc - yc;

        return Mathf.Sqrt(dX * dX + dY * dY);
    }

    // All the points belonging to a circle have the same speed, which is the speed
    // of the center of the circle:
    public Vector2 VelocityOfPoint(float x, float y) {
        return new Vector2(xcP, ycP);
    }
}
