using UnityEngine;

public class Circle {
    // Center and radius of the circle:
    private float xc, yc, R;

    public Circle(float xc, float yc, float R) {
        this.xc = xc;
        this.yc = yc;
        this.R = R;
    }

    public void DrawGizmos() {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(new Vector3(xc, 0.5f, yc), R);
    }
}
