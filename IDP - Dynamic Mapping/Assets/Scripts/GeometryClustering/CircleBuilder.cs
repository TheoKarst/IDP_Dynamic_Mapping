using System.Collections.Generic;
using UnityEngine;

public class CircleBuilder {

    // Points representing this circle:
    private List<Point> points;

    // Regression parameters of the circle:
    private float Rx = 0, Ry = 0;

    // Current center of the circle:
    private float xc, yc;

    public CircleBuilder() {
        points = new List<Point>();
    }

    public CircleBuilder(List<Point> points) {
        this.points = points;

        foreach (Point point in points) {
            Rx += point.x;
            Ry += point.y;
        }

        int n = points.Count;
        xc = Rx / n;
        yc = Ry / n;
    }

    public void AddPoint(Point point) {
        points.Add(point);

        Rx += point.x;
        Ry += point.y;

        int n = points.Count;
        xc = Rx / n;
        yc = Ry / n;
    }

    public Circle Build() {
        float R = ComputeCircleRadius();

        return new Circle(xc, yc, R);             
    }

    private float ComputeCircleRadius() {
        int n = points.Count;

        if (n == 1)
            return 0;

        float sigma_x2 = 0, sigma_y2 = 0;
        foreach (Point p in points) {
            sigma_x2 += (p.x - xc) * (p.x - xc);
            sigma_y2 += (p.y - yc) * (p.y - yc);
        }

        // The division by 'n-1' instead of 'n' is known as Bessel's correction,
        // which corrects the bias in the estimation of the points variance:
        sigma_x2 /= n - 1; sigma_y2 /= n - 1;

        return Mathf.Sqrt(sigma_x2 + sigma_y2);
    }

    // Return the distance between the circle center and the given point:
    public float DistanceFrom(Point point) {
        float dX = point.x - xc;
        float dY = point.y - yc;

        return Mathf.Sqrt(dX * dX + dY * dY);
    }

    public int PointsCount() {
        return points.Count;
    }
}
