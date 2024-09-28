using System.Collections.Generic;
using System.Net;
using UnityEngine;

/// <summary>
/// Class used to represent circle primitives for the mapping using geometric primitives
/// </summary>
public class Circle {
    
    // Color of the circle (for drawing):
    public Color circleColor = Color.red;

    // Center and radius of the circle:
    private Vector2 _center;
    public Vector2 center { get => _center; }
    private float R;

    // Estimated derivative of center.x and center.y with respect to time:
    private Vector2 speed;

    // If the circle is consistent with the current observations:
    public bool isValid = true;

    /// <summary>
    /// Instantiates a circle with the given center and radius
    /// </summary>
    /// <param name="center">World position of the circle</param>
    /// <param name="R">Radius of the circle in meters</param>
    public Circle(Vector2 center, float R) {
        this._center = center;
        this.R = R;

        this.speed = Vector2.zero;
    }

    /// <summary>
    /// Draws the circle in the scene using Unity gizmos
    /// </summary>
    /// <param name="height">Position of the circle along the Z-axis</param>
    public void DrawGizmos(float height, bool drawSpeedEstimate) {
        Gizmos.color = circleColor;
        Gizmos.DrawSphere(Utils.To3D(_center, height), R);

        // Draw an arrow representing the estimated speed of the circle:
        if (drawSpeedEstimate) {
            Vector3 p1 = Utils.To3D(_center, height);
            Vector3 p2 = Utils.To3D(_center + 10 * speed, height);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(p1, p2);
        }
    }

    /// <summary>
    /// From the previous circle estimate and the elapsed time since the last update,
    /// predict where the circle should be now
    /// </summary>
    /// <param name="elapsedTime">Elapsed time in seconds since the last update</param>
    /// <param name="friction">Friction applied to the speed of the circle (between 0 and 1)</param>
    public void PredictState(float elapsedTime, float friction) {
        _center += elapsedTime * speed;
        speed *= 1 - friction;
    }

    /// <summary>
    /// Supposing that this circle belongs to the current model of the environment, use the
    /// given circle (that is supposed to be matched with this one) to update the center and
    /// radius of this circle
    /// </summary>
    /// <param name="other">Observed circle matched with this one</param>
    public void UpdateCircleUsingMatch(Circle other) {
        Vector2 newCenter = (_center + other._center) / 2;
        R = (R + other.R) / 2;

        // Use the newCenter to update the circle speed estimate, using a simple
        // exponential low pass filter (TODO: Implement Kalman Filter instead):
        const float m = 0.9f;
        speed = m * speed + (1 - m) * (newCenter - _center);

        // Update the center of the circle:
        _center = newCenter;
    }

    /// <summary>
    /// Computes the Euclidean distance between the centers of this circle and the
    /// center of the given circle
    /// </summary>
    public float DistanceFrom(Circle other) {
        return (other._center - _center).magnitude;
    }

    /// <summary>
    /// Returns if the circle is far enough from the given lines
    /// </summary>
    /// <param name="lines">List of lines</param>
    /// <param name="minDistanceToLines">Minimum allowed orthogonal distance 
    /// between a line and the center of the circle</param>
    /// <returns></returns>
    public bool IsFarFromLines(List<DynamicLine> lines, float minDistanceToLines) {
        if (lines == null)
            return true;

        foreach (DynamicLine line in lines)
            if (line.DistanceOf(_center) < minDistanceToLines)
                return false;

        return true;
    }
}
