using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;

public class LineBuilder {
    // Matrix builder used as a shortcut for matrix creation:
    private static MatrixBuilder<double> M = Matrix<double>.Build;
    
    // Points representing this line:
    private List<Point> points = new List<Point>();

    // Regression parameters of the line:
    private float Rx, Ry, Rxx, Ryy, Rxy;

    // Parameters defining the line:
    private float rho, theta;

    // Current endpoints of the line, and a flag saying if they are up to date:
    private Vector2 beginPoint, endPoint;
    private bool upToDateEndpoints = false;

    public LineBuilder(Point initialPoint) {
        points.Add(initialPoint);

        Rx = initialPoint.position.x;
        Ry = initialPoint.position.y;
        Rxx = initialPoint.position.x * initialPoint.position.x;
        Ryy = initialPoint.position.y * initialPoint.position.y;
        Rxy = initialPoint.position.x * initialPoint.position.y;
    }

    // Add a point to the line, and update the line parameters:
    public void AddPoint(Point point) {
        points.Add(point);

        // Now the endpoints are not up to date anymore:
        upToDateEndpoints = false;

        Rx += point.position.x;
        Ry += point.position.y;
        Rxx += point.position.x * point.position.x;
        Ryy += point.position.y * point.position.y;
        Rxy += point.position.x * point.position.y;

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

    // Get the last point that was added to the line (cannot be null,
    // since a line always contains at least one point):
    public Point GetLastPoint() {
        return points[points.Count - 1];
    }

    // Return the length of the line, using its endpoints
    public float Length() {
        if(!upToDateEndpoints)
            UpdateEndpoints();

        return Vector2.Distance(beginPoint, endPoint);
    }

    // Convert this line into a circle cluster (this line shouldn't be used anymore after that):
    public CircleBuilder ToCircle() {
        return new CircleBuilder(points);
    }

    public DynamicLine Build(Matrix<double> Q) {
        // Compute the endpoints and covariance matrix of the line:
        if (!upToDateEndpoints) 
            UpdateEndpoints();

        Matrix<double> covariance = ComputeCovariance();

        // Build the line:
        DynamicLine line = new DynamicLine(rho, theta, covariance, beginPoint, endPoint, Q);

        // Match all the points to the built line:
        foreach (Point point in points) point.MatchToPrimitive(line);

        return line;
    }

    // Compute the endpoints of the line by projecting the first and last points that were added
    // in the line builder, along the line defined by the parameters (rho, theta):
    private void UpdateEndpoints() {
        float costheta = Mathf.Cos(theta), sintheta = Mathf.Sin(theta);
        // Unit vector along the line:
        Vector2 u = new Vector2(-sintheta, costheta);        

        // Compute the projection of the first point and last point of the line:
        float pFirst = Vector2.Dot(u, points[0].position);
        float pLast = Vector2.Dot(u, points[points.Count-1].position);

        // "Center" of the infinite line:
        Vector2 center = new Vector2(rho * costheta, rho * sintheta);

        beginPoint = center + pFirst * u;
        endPoint = center + pLast * u;
        upToDateEndpoints = true;
    }

    // Compute the covariance matrix of the parameters (rho, theta) of the line:
    private Matrix<double> ComputeCovariance() {
        // Step 1: Compute Cv (covariance matrix on m,q or s,t):
        Matrix<double> Cv = M.Dense(2, 2, 0);    // 2*2 Zero Matrix

        int n = points.Count;
        float N1 = Rxx * n - Rx * Rx;
        float N2 = Ryy * n - Ry * Ry;
        float T = Rxy * n - Rx * Ry;

        Matrix<double> Hl;
        if (N1 >= N2) {     // We use (m,q) representation
            float m = T / N1;
            float q = (Ry - m * Rx) / n;

            for (int i = 0; i < points.Count; i++) {
                Matrix<double> J = JacobianMQi(m, N1, T, i);
                Matrix<double> Cp = points[i].Cp;
                Cv += J * Cp.TransposeAndMultiply(J);
            }

            // Compute Hl, Jacobian of (rho, theta) with respect to (m, q):
            float tmp = 1 + m * m;
            float drho_dm = -m * Mathf.Abs(q) / Mathf.Pow(tmp, 1.5f);
            float drho_dq = Mathf.Sign(q) / Mathf.Sqrt(tmp);
            float dtheta_dm = 1 / tmp;
            float dtheta_dq = 0;

            Hl = M.DenseOfArray(new double[,] {
                { drho_dm,      drho_dq },
                { dtheta_dm,    dtheta_dq } });
        }
        else {              // We use (s,t) representation
            float s = T / N2;
            float t = (Rx - s * Ry) / n;

            for (int i = 0; i < points.Count; i++) {
                Matrix<double> J = JacobianSTi(s, N2, T, i);
                Matrix<double> Cp = points[i].Cp;
                Cv += J * Cp.TransposeAndMultiply(J);
            }

            // Compute Hl, Jacobian of (rho, theta) with respect to (s, t):
            float tmp = 1 + s * s;
            float drho_ds = -s * Mathf.Abs(t) / Mathf.Pow(tmp, 1.5f);
            float drho_dt = Mathf.Sign(t) / Mathf.Sqrt(tmp);
            float dtheta_ds = -1 / tmp;
            float dtheta_dt = 0;

            Hl = M.DenseOfArray(new double[,] {
                { drho_ds,      drho_dt },
                { dtheta_ds,    dtheta_dt } });
        }

        // Setp 2: Use the previously computed Cv and Hl to compute the covariance
        // matrix of the parameters (rho, theta) of the line:
        return Hl * Cv.TransposeAndMultiply(Hl);
    }

    private Matrix<double> JacobianMQi(float m, float N1, float T, int i) {
        int n = points.Count;
        float xpi = points[i].position.x, ypi = points[i].position.y;

        float dm_dx = (N1 * (n * ypi - Ry) - 2 * T * (n * xpi - Rx)) / (N1 * N1);
        float dm_dy = (n * xpi - Rx) / N1;
        float dq_dx = -(Rx * dm_dx + m) / n;
        float dq_dy = (1 - Rx * dm_dy) / n;

        return M.DenseOfArray(new double[,] { 
            { dm_dx, dm_dy }, 
            { dq_dx, dq_dy } });
    }

    private Matrix<double> JacobianSTi(float s, float N2, float T, int i) {
        int n = points.Count;
        float xpi = points[i].position.x, ypi = points[i].position.y;

        float ds_dx = (n * ypi - Ry) / N2;
        float ds_dy = (N2 * (n * xpi - Rx) - 2 * T * (n * ypi - Ry)) / (N2 * N2);
        float dt_dx = (1 - Ry * ds_dx) / n;
        float dt_dy = -(Ry * ds_dy + s) / n;

        return M.DenseOfArray(new double[,] {
            { ds_dx, ds_dy },
            { dt_dx, dt_dy } });
    }

    // Return the distance between the line and the given point:
    public float DistanceFrom(Point point) {
        return Mathf.Abs(point.position.x * Mathf.Cos(theta) + point.position.y * Mathf.Sin(theta) - rho);
    }

    public int PointsCount() {
        return points.Count;
    }
}
