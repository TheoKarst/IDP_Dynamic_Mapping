using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class LineBuilder {
    // Matrix and vector builders used as a shortcut for matrix and vector creation:
    private static MatrixBuilder<float> M = Matrix<float>.Build;
    private static VectorBuilder<float> V = Vector<float>.Build;

    // Points representing this line:
    private List<Point> points = new List<Point>();

    // Regression parameters of the line:
    private float Rx, Ry, Rxx, Ryy, Rxy;

    // Parameters defining the line:
    private float rho, theta;

    // Endpoints of the line, as well as a flag saying if they are up to date:
    private Vector2 beginPoint, endPoint;
    private bool uptodateEndpoints = false;

    // Covariance matrix of the parameters (rho, theta) of the line:
    private Matrix<float> covariance = null;

    public LineBuilder(Point initialPoint) {
        points.Add(initialPoint);

        Rx = initialPoint.x;
        Ry = initialPoint.y;
        Rxx = initialPoint.x * initialPoint.x;
        Ryy = initialPoint.y * initialPoint.y;
        Rxy = initialPoint.x * initialPoint.y;
    }

    public void DrawGizmos() {
        if (!uptodateEndpoints || covariance == null)
            Debug.LogError("Trying to draw a line that is not correctly built");

        else {
            Vector3 p1 = new Vector3(beginPoint.x, 0.5f, beginPoint.y);
            Vector3 p2 = new Vector3(endPoint.x, 0.5f, endPoint.y);
            Handles.DrawBezier(p1, p2, p1, p2, Color.red, null, 4);
        }        
    }

    // Add a point to the line, and update the line parameters:
    public void AddPoint(Point point) {
        points.Add(point);

        // Cancel the computation of the line covariance matrix and endpoints, as they are
        // not valid anymore:
        uptodateEndpoints = false;
        covariance = null;

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

    // Return if the otther line (supposed to be part of the current world model) is a good candidate
    // to be matched with this line. If this is the case, we will have to check the norm distance
    // between the lines as a next step:
    public bool IsMatchCandidate(LineBuilder other, float maxAngleDistance, float maxEndpointDistance) {
        // If the angular difference between both lines is too big, the lines cannot match:
        if(Mathf.Abs(theta - other.theta) > maxAngleDistance)
            return false;

        // If the distance of the endpoints of this line to the 
        if(other.DistanceFrom(beginPoint) > maxEndpointDistance)
            return false;

        if(other.DistanceFrom(endPoint) > maxEndpointDistance)
            return false;

        // If both lines are close enough, then the other line should ba a good candidate for
        // the matching:
        return true;
    }

    // Compute the Mahalanobis distance between this line and the given one:
    public float ComputeNormDistance(LineBuilder other) {
        Vector<float> Xl = V.DenseOfArray(new float[] {rho, theta});
        Vector<float> Xm = V.DenseOfArray(new float[] {other.rho, other.theta});
        Matrix<float> Cl = this.covariance;
        Matrix<float> Cm = other.covariance;

        Vector<float> X = Xl - Xm;

        return (X.ToRowMatrix() * (Cl + Cm).Inverse() * X)[0];
    }

    // Supposing that this line belongs to the current model of the environment, use the given 
    // line (that was supposed to be matched with this one) to update this line position estimate,
    // covariance matrix and endpoints:
    public void UpdateLineUsingMatching(LineBuilder other) {
        // Perform some renamings to match the paper description:
        Vector<float> Xm = V.DenseOfArray(new float[] { rho, theta });
        Vector<float> Xl = V.DenseOfArray(new float[] { other.rho, other.theta });
        Matrix<float> Cm = this.covariance;
        Matrix<float> Cl = other.covariance;

        // Use a static Kalman Filter to update this line covariance and state (rho, theta) estimate:
        Matrix<float> K = Cl * (Cl + Cm).Inverse();
        Vector<float> Xr = Xl + K * (Xm - Xl);
        Matrix<float> Cr = Cl - K * Cl;

        // Update this line (rho, theta) parameters, and covariance matrix:
        rho = Xr[0]; theta = Xr[1];
        covariance = Cr;
    }

    // Return the distance between the line and the given point:
    public float DistanceFrom(Point point) {
        return Mathf.Abs(point.x * Mathf.Cos(theta) + point.y * Mathf.Sin(theta) - rho);
    }

    // Return the distance between the line and the given point:
    public float DistanceFrom(Vector2 point) {
        return Mathf.Abs(point.x * Mathf.Cos(theta) + point.y * Mathf.Sin(theta) - rho);
    }

    public int PointsCount() {
        return points.Count;
    }

    // Get the last point that was added to the line (canno be null, since a line always contains at least one point):
    public Point GetLastPoint() {
        return points[points.Count - 1];
    }

    // Return the length of the line, using its endpoints
    public float Length() {
        if(!uptodateEndpoints)
            ComputeEndpoints();

        return Vector2.Distance(beginPoint, endPoint);
    }

    // Convert this line into a circle cluster (this line shouldn't be used anymore after that):
    public Circle ToCircle() {
        return new Circle(points);
    }

    public void Build() {
        if (!uptodateEndpoints) ComputeEndpoints();
        if (covariance == null) ComputeCovariance();
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
        uptodateEndpoints = true;
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
        covariance = Hl * Cv.TransposeAndMultiply(Hl);
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
