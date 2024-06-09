using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public struct Point {
    public bool isValid;
    public float x, y;          // Estimated position of the point, from the Kalman Filter
    private float angle;        // Angle of the point in radians, from the LIDAR

    // Covariance matrix for the position (x, y) of the point:
    public readonly Matrix<double> Cp;

    // Primitive this point is supposed to belong to:
    private Primitive matchingPrimitive;

    public Point(float x, float y, float angle, Matrix<double> covariance, bool isValid) {
        this.x = x;
        this.y = y;
        this.angle = angle;

        this.Cp = covariance;
        matchingPrimitive = null;
        this.isValid = isValid;
    }

    public void DrawGizmos() {
        // Center of the point in Unity 3D world space:
        Vector3 center = new Vector3(x, 0.2f, y);

        // Draw a sphere at the position of the point:
        Gizmos.color = Color.yellow;
        // Gizmos.DrawSphere(center, 0.01f);

        // Draw a cube at the position of the point, representing its eror estimate:
        Gizmos.DrawCube(center, new Vector3(Mathf.Sqrt((float) Cp[0, 0]), 0, Mathf.Sqrt((float) Cp[1, 1])));

        // If the point is matched with a primitive, draw a line, representing the speed
        // estimate of the point:
        if(matchingPrimitive != null) {
            Vector2 speed = 10 * matchingPrimitive.VelocityOfPoint(x, y);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(center, new Vector3(center.x + speed.x, center.y, center.z + speed.y));
        }
    }

    public static float Dist(Point a, Point b) {
        float dX = a.x - b.x;
        float dY = a.y - b.y;
        
        return Mathf.Sqrt(dX * dX + dY * dY);
    }

    public static float AngularDifference(Point a, Point b) {
        return Utils.DeltaAngleRadians(a.angle, b.angle);
    }

    public void MatchToPrimitive(Primitive primitive) {
        matchingPrimitive = primitive;
    }
}