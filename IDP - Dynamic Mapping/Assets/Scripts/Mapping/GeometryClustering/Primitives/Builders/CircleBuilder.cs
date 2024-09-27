using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Class used to build circles from observed points
/// </summary>
public class CircleBuilder {
    
    // Points representing this circle:
    private List<Point> points;

    // Regression parameters of the circle (Rx, Ry):
    private Vector2 Rxy = new Vector2();

    // Current center of the circle:
    private Vector2 center;

    /// <summary>
    /// Instantiates a circle builder, to build circles from observed points
    /// </summary>
    public CircleBuilder() {
        points = new List<Point>();
    }

    /// <summary>
    /// Instantiates a circle builder, to build circles from observed points
    /// </summary>
    /// <param name="points">The points to add to the current circle</param>
    public CircleBuilder(List<Point> points) {
        this.points = points;

        foreach (Point point in points)
            Rxy += point.position;

        center = Rxy / points.Count;
    }

    /// <summary>
    /// Adds a point to the circle to build
    /// </summary>
    /// <param name="point">The point to add</param>
    public void AddPoint(Point point) {
        points.Add(point);

        Rxy += point.position;

        center = Rxy / points.Count;
    }

    /// <summary>
    /// Builds a circle from the points added to this circle builder
    /// </summary>
    /// <returns>The circle that was built</returns>
    public Circle Build() {
        // Compute the radius of the circle:
        float R = ComputeCircleRadius();

        // Build the circle:
        Circle circle = new Circle(center, R);

        return circle;
    }

    /// <summary>
    /// Computes the radius of the circle from the position of the points in the circle builder
    /// </summary>
    /// <returns>The radius of the circle</returns>
    private float ComputeCircleRadius() {
        int n = points.Count;

        if (n == 1)
            return 0;

        float sigma_x2 = 0, sigma_y2 = 0;
        foreach (Point p in points) {
            sigma_x2 += (p.position.x - center.x) * (p.position.x - center.x);
            sigma_y2 += (p.position.y - center.y) * (p.position.y - center.y);
        }

        // The division by 'n-1' instead of 'n' is known as Bessel's correction,
        // which corrects the bias in the estimation of the points variance:
        sigma_x2 /= n - 1; sigma_y2 /= n - 1;

        return Mathf.Sqrt(sigma_x2 + sigma_y2);
    }

    /// <summary>
    /// Returns teh distance between the circle center and the given point
    /// </summary>
    public float DistanceFrom(Point point) {
        return (point.position - center).magnitude;
    }

    /// <summary>
    /// Returns the number of points currently in the circle
    /// </summary>
    public int PointsCount() {
        return points.Count;
    }
}
