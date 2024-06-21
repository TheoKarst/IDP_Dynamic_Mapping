using System.Collections.Generic;
using UnityEngine;

public class CircleBuilder {

    // Points representing this circle:
    private List<Point> points;

    // Regression parameters of the circle (Rx, Ry):
    private Vector2 Rxy = new Vector2();

    // Current center of the circle:
    private Vector2 center;

    public CircleBuilder() {
        points = new List<Point>();
    }

    public CircleBuilder(List<Point> points) {
        this.points = points;

        foreach (Point point in points)
            Rxy += point.position;

        center = Rxy / points.Count;
    }

    public void AddPoint(Point point) {
        points.Add(point);

        Rxy += point.position;

        center = Rxy / points.Count;
    }

    public Circle Build() {
        // Compute the radius of the circle:
        float R = ComputeCircleRadius();

        // Build the circle:
        Circle circle = new Circle(center.x, center.y, R);

        // Match all the points to the built circle:
        foreach (Point point in points) point.MatchToPrimitive(circle);

        return circle;
    }

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

    // Return the distance between the circle center and the given point:
    public float DistanceFrom(Point point) {
        return (point.position - center).magnitude;
    }

    public int PointsCount() {
        return points.Count;
    }
}
