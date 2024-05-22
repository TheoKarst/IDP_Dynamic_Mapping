using System.Collections.Generic;
using UnityEngine;

class Line {

    // Points representing this line:
    private List<Point> points = new List<Point>();

    // Regression parameters of the line:
    private float Rx = 0, Ry = 0, Rxx = 0, Ryy = 0, Rxy = 0;

    // Parameters defining the line:
    private float rho, theta;

    // Endpoints of the line:
    private Vector2 beginPoint, endPoint;

    public void Draw() {
        // TODO: Implement this
    }

    // Add a point to the line, and update the line parameters:
    public void AddPoint(Point point) {
        points.Add(point);

        Rx += point.x;
        Ry += point.y;
        Rxx += point.x * point.x;
        Ryy += point.y * point.y;
        Rxy += point.x * point.y;

        int n = points.Count;
        float N1 = Rxx * n - Rx * Rx;
        float N2 = Ryy * n - Ry * Ry;
        float T = Rxy * n - Rx * Ry;

        // N1 and N2 represent the width of the cloud of the regression points along the X- and Y-axis. If N1 is larger 
        // than N2, the cloud of points lies more horizontally than vertically which makes regression of y to x (y = mx + q)
        // more favourable. Otherwise, the regression of x to y is selected (x = sy + t):
        if (N1 >= N2) {
            float m = T / N1;
            float q = (Ry - m * Rx) / n;

            rho = Mathf.Abs(q / Mathf.Sqrt(m * m + 1));
            theta = Mathf.Atan2(q, -q * m);
        }
        else {
            float s = T / N2;
            float t = (Rx - s * Ry) / n;

            rho = Mathf.Abs(t / Mathf.Sqrt(s * s + 1));
            theta = Mathf.Atan2(-t * s, t);
        }
    }

    // Return the distance between the line and the given point:
    public float DistanceFrom(Point point) {
        return Mathf.Abs(point.x * Mathf.Cos(theta) + point.y * Mathf.Sin(theta) - rho);
    }

    public Point GetLastPoint() {
        int n = points.Count;
        return n == 0 ? null : points[n - 1];
    }

    public int PointsCount() {
        return points.Count;
    }

    public void ComputeEndpoints() {
        float costheta = Mathf.Cos(theta), sintheta = Mathf.Sin(theta);
        float x = rho * costheta;
        float y = rho * sintheta;

        // Unit vector along the line:
        Vector2 u = new Vector2(-sintheta, costheta);        

        // Compute the projection of the first point and last point of the line:
        Point firstPoint = points[0];
        Point lastPoint = points[points.Count - 1];

        float pFirst = u.x * (firstPoint.x - x) + u.y * (firstPoint.y - y);
        float pLast = u.x * (lastPoint.x - x) + u.y * (lastPoint.y - y);

        beginPoint = new Vector2(x + pFirst * u.x, y + pFirst * u.y);
        endPoint = new Vector2(x + pLast * u.x, y + pLast * u.y);
    }
}
