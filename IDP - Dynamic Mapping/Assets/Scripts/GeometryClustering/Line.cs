using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

class Line {
    // Matrix builder used as a shortcut for matrix creation:
    private static MatrixBuilder<float> M = Matrix<float>.Build;

    // Points representing this line:
    private List<Point> points = new List<Point>();

    // Regression parameters of the line:
    private float Rx = 0, Ry = 0, Rxx = 0, Ryy = 0, Rxy = 0;

    // Parameters defining the line:
    private float rho, theta;

    // Endpoints of the line:
    private Vector2 beginPoint, endPoint;

    // Covariance matrix of the parameters (rho, theta) of the line:
    private Matrix<float> Cl;

    public void DrawGizmos() {
        //Gizmos.color = Color.red;
        //Gizmos.DrawLine(new Vector3(beginPoint.x, 0.5f, beginPoint.y), 
        //                new Vector3(endPoint.x, 0.5f, endPoint.y));

        Vector3 p1 = new Vector3(beginPoint.x, 0.5f, beginPoint.y);
        Vector3 p2 = new Vector3(endPoint.x, 0.5f, endPoint.y);
        Handles.DrawBezier(p1, p2, p1, p2, Color.red, null, 4);
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

    public void Build() {
        // Compute the endpoints of the line, and the covariance matrix of the
        // parameters (rho, theta) of the line, depending on the covariance matrices
        // of the points belonging to the line:

        ComputeEndpoints();
        ComputeCovariance();
    }

    private void ComputeEndpoints() {
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

    private void ComputeCovariance() {
        // Step 1: Compute Cv (covariance matrix on m,q or s,t):
        Matrix<float> Cv = M.Dense(2, 2, 0);    // 2*2 Zero Matrix

        int n = points.Count;
        float N1 = Rxx * n - Rx * Rx;
        float N2 = Ryy * n - Ry * Ry;
        float T = Rxy * n - Rx * Ry;

        Matrix<float> Hl;
        if (N1 >= N2) {     // We use (m,q) representation
            float m = T / N1;
            float q = (Ry - m * Rx) / n;

            for (int i = 0; i < points.Count; i++) {
                Matrix<float> J = JacobianMQi(m, N1, T, i);
                Matrix<float> Cp = points[i].Cp;
                Cv += J * Cp.TransposeAndMultiply(J);
            }

            // Compute Hl, Jacobian of (rho, theta) with respect to (m, q):
            float tmp = 1 + m * m;
            float drho_dm = -m * Mathf.Abs(q) / Mathf.Pow(tmp, 1.5f);
            float drho_dq = Mathf.Sign(q) / Mathf.Sqrt(tmp);
            float dtheta_dm = 1 / tmp;
            float dtheta_dq = 0;

            Hl = M.DenseOfArray(new float[,] {
                { drho_dm,      drho_dq },
                { dtheta_dm,    dtheta_dq } });
        }
        else {              // We use (s,t) representation
            float s = T / N2;
            float t = (Rx - s * Ry) / n;

            for (int i = 0; i < points.Count; i++) {
                Matrix<float> J = JacobianSTi(s, N2, T, i);
                Matrix<float> Cp = points[i].Cp;
                Cv += J * Cp.TransposeAndMultiply(J);
            }

            // Compute Hl, Jacobian of (rho, theta) with respect to (s, t):
            float tmp = 1 + s * s;
            float drho_ds = -s * Mathf.Abs(t) / Mathf.Pow(tmp, 1.5f);
            float drho_dt = Mathf.Sign(t) / Mathf.Sqrt(tmp);
            float dtheta_ds = -1 / tmp;
            float dtheta_dt = 0;

            Hl = M.DenseOfArray(new float[,] {
                { drho_ds,      drho_dt },
                { dtheta_ds,    dtheta_dt } });
        }

        // Setp 2: Use the previously computed Cv and Hl to compute the covariance
        // matrix of the parameters (rho, theta) of the line:
        Cl = Hl * Cv.TransposeAndMultiply(Hl);
    }

    private Matrix<float> JacobianMQi(float m, float N1, float T, int i) {
        int n = points.Count;
        float xpi = points[i].x, ypi = points[i].y;

        float dm_dx = (N1 * (n * ypi - Ry) - 2 * T * (n * xpi - Rx)) / (N1 * N1);
        float dm_dy = (n * xpi - Rx) / N1;
        float dq_dx = -(Rx * dm_dx + m) / n;
        float dq_dy = (1 - Rx * dm_dy) / n;

        return M.DenseOfArray(new float[,] { 
            { dm_dx, dm_dy }, 
            { dq_dx, dq_dy } });
    }

    private Matrix<float> JacobianSTi(float s, float N2, float T, int i) {
        int n = points.Count;
        float xpi = points[i].x, ypi = points[i].y;

        float ds_dx = (n * ypi - Ry) / N2;
        float ds_dy = (N2 * (n * xpi - Rx) - 2 * T * (n * ypi - Ry)) / (N2 * N2);
        float dt_dx = (1 - Ry * ds_dx) / n;
        float dt_dy = -(Ry * ds_dy + s) / n;

        return M.DenseOfArray(new float[,] {
            { ds_dx, ds_dy },
            { dt_dx, dt_dy } });
    }
}
