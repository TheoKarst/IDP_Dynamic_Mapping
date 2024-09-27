using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

/// <summary>
/// Class used to represent points oberved by the LIDAR, for the mapping using geometric primitives
/// </summary>
public class Point {
    
    public Vector2 position;    // Estimated position of the point, from the Kalman Filter
    public float angle;         // Angle of the point in radians, from the LIDAR

    // Covariance matrix for the position (x, y) of the point:
    public readonly Matrix<double> Cp;

    /// <summary>
    /// Instantiates a point at the given position (x, y)
    /// </summary>
    /// <param name="angle">Angle in radians between the LIDAR and the point</param>
    /// <param name="covariance">2x2 covariance matrix of the point</param>
    public Point(float x, float y, float angle, Matrix<double> covariance) {
        this.position = new Vector2(x, y);
        this.angle = angle;

        this.Cp = covariance;
    }

    /// <summary>
    /// Draws the point in the scene using Unity Gizmos
    /// </summary>
    /// <param name="height">Position of the point along the Z-axis</param>
    /// <param name="drawError">If true, draws a cube representing the error estimate on the point</param>
    public void DrawGizmos(float height, bool drawError) {
        // Center of the point in Unity 3D world space:
        Vector3 center = Utils.To3D(position, height);

        Gizmos.color = Color.yellow;

        // Draw a cube at the position of the point, representing its eror estimate:
        if (drawError)
            Gizmos.DrawCube(center, new Vector3(Mathf.Sqrt((float)Cp[0, 0]), 0, Mathf.Sqrt((float)Cp[1, 1])));

        // Draw a small sphere at the position of the point:
        else
            Gizmos.DrawSphere(center, 0.01f);
    }

    /// <summary>Returns the Euclidean distance between two points</summary>
    public static float Dist(Point a, Point b) {
        return (b.position - a.position).magnitude;
    }

    /// <summary>Returns the angular difference from the LIDAR between two points</summary>
    public static float AngularDifference(Point a, Point b) {
        return Utils.DeltaAngleRadians(a.angle, b.angle);
    }
}