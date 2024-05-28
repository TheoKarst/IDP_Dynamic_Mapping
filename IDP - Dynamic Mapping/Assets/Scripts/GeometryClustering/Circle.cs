using System.Collections.Generic;
using UnityEngine;

public class Circle {

    // Points representing this circle:
    private List<Point> points;

    // Regression parameters of the circle:
    private float Rx = 0, Ry = 0;

    // Center and radius of the circle (R < 0 means it is not up to date):
    private float xc, yc, R = -1;

    public Circle() {
        points = new List<Point>();
    }

    public Circle(List<Point> points) {
        this.points = points;

        foreach (Point point in points) {
            Rx += point.x;
            Ry += point.y;
        }

        int n = points.Count;
        xc = Rx / n;
        yc = Ry / n;
    }

    public void DrawGizmos() {
        if (R < 0)
            Debug.LogError("Trying to draw a line that is not correctly built");

        else {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(new Vector3(xc, 0.5f, yc), R);
        }
    }

    public void AddPoint(Point point) {
        points.Add(point);

        Rx += point.x;
        Ry += point.y;

        int n = points.Count;
        xc = Rx / n;
        yc = Ry / n;

        // Cancel the computation of the circle radius as it's not valid anymore:
        R = -1;
    }

    // Compute the circle radius:
    public void Build() {
        int n = points.Count;

        if (n == 1) {
            R = 0;
        }
        else {
            float sigma_x2 = 0, sigma_y2 = 0;
            foreach (Point p in points) {
                sigma_x2 += (p.x - xc) * (p.x - xc);
                sigma_y2 += (p.y - yc) * (p.y - yc);
            }

            // The division by 'n-1' instead of 'n' is known as Bessel's correction,
            // which corrects the bias in the estimation of the points variance:
            sigma_x2 /= n - 1; sigma_y2 /= n - 1;

            R = Mathf.Sqrt(sigma_x2 + sigma_y2);
        }        
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
