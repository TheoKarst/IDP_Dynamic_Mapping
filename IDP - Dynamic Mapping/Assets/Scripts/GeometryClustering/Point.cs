using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

class Point {
    public float x;
    public float y;
    public float angle;

    // Covariance matrix for the position (x, y) of the point:
    private Matrix<float> covariance;

    public Point(float x, float y, float angle, Matrix<float> covariance) {
        this.x = x;
        this.y = y;
        this.angle = angle;

        this.covariance = covariance;
    }

    public void DrawGizmos() {
        Gizmos.color = Color.yellow;
        // Gizmos.DrawSphere(new Vector3(x, 0.5f, y), 0.04f);

        Gizmos.DrawCube(new Vector3(x, 0.5f, y),
                        new Vector3(Mathf.Sqrt(covariance[0, 0]), 0, Mathf.Sqrt(covariance[1, 1])));
    }

    public static float Dist(Point a, Point b) {
        float dX = a.x - b.x;
        float dY = a.y - b.y;
        
        return Mathf.Sqrt(dX * dX + dY * dY);
    }

    public static float AngularDifference(Point a, Point b) {
        return Mathf.Abs(a.angle - b.angle);
    }
}