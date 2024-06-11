using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public struct Point {
    public bool isValid;
    public Vector2 position;    // Estimated position of the point, from the Kalman Filter
    public float angle;         // Angle of the point in radians, from the LIDAR

    // Covariance matrix for the position (x, y) of the point:
    public readonly Matrix<double> Cp;

    // Primitive this point is supposed to belong to:
    private Primitive matchingPrimitive;

    public Point(float x, float y, float angle) {
        this.position = new Vector2(x, y);
        this.angle = angle;

        this.Cp = null;
        this.matchingPrimitive = null;
        this.isValid = true;
    }

    public Point(float x, float y, float angle, Matrix<double> covariance, bool isValid) {
        this.position = new Vector2(x, y);
        this.angle = angle;

        this.Cp = covariance;
        this.matchingPrimitive = null;
        this.isValid = isValid;
    }

    public void DrawGizmos(float height) {
        // Center of the point in Unity 3D world space:
        Vector3 center = Utils.To3D(position, height);

        // Draw a sphere at the position of the point:
        Gizmos.color = Color.yellow;
        // Gizmos.DrawSphere(center, 0.01f);

        // Draw a cube at the position of the point, representing its eror estimate:
        Gizmos.DrawCube(center, new Vector3(Mathf.Sqrt((float) Cp[0, 0]), 0, Mathf.Sqrt((float) Cp[1, 1])));

        // If the point is matched with a primitive, draw a line, representing the speed
        // estimate of the point:
        if(matchingPrimitive != null) {
            Vector2 speed = 10 * matchingPrimitive.VelocityOfPoint(position.x, position.y);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(center, Utils.To3D(position + speed, height));
        }
    }

    public static float Dist(Point a, Point b) {
        return (b.position - a.position).magnitude;
    }

    public static float AngularDifference(Point a, Point b) {
        return Utils.DeltaAngleRadians(a.angle, b.angle);
    }

    public void MatchToPrimitive(Primitive primitive) {
        matchingPrimitive = primitive;
    }
}