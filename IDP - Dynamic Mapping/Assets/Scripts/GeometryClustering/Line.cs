using MathNet.Numerics.LinearAlgebra;
using UnityEditor;
using UnityEngine;

public class Line {
    // Matrix and vector builders used as a shortcut for matrix and vector creation:
    private static MatrixBuilder<float> M = Matrix<float>.Build;
    private static VectorBuilder<float> V = Vector<float>.Build;

    private float rho, theta;
    private Matrix<float> covariance;

    private Vector2 beginPoint, endPoint;

    public Line(float rho, float theta, Matrix<float> covariance, Vector2 beginPoint, Vector2 endPoint) {
        this.rho = rho;
        this.theta = theta;
        this.covariance = covariance;
        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
    }

    public void DrawGizmos() {
        Vector3 p1 = new Vector3(beginPoint.x, 0.5f, beginPoint.y);
        Vector3 p2 = new Vector3(endPoint.x, 0.5f, endPoint.y);
        Handles.DrawBezier(p1, p2, p1, p2, Color.red, null, 4);
    }

    // Return if the otther line (supposed to be part of the current world model) is a good candidate
    // to be matched with this line. If this is the case, we will have to check the norm distance
    // between the lines as a next step:
    public bool IsMatchCandidate(Line other, float maxAngleDistance, float maxEndpointDistance) {
        // If the angular difference between both lines is too big, the lines cannot match:
        if (Mathf.Abs(theta - other.theta) > maxAngleDistance)
            return false;

        // If the distance of the endpoints of this line to the 
        if (other.DistanceFrom(beginPoint) > maxEndpointDistance)
            return false;

        if (other.DistanceFrom(endPoint) > maxEndpointDistance)
            return false;

        // If both lines are close enough, then the other line should ba a good candidate for
        // the matching:
        return true;
    }

    // Return the distance between the line and the given point:
    public float DistanceFrom(Vector2 point) {
        return Mathf.Abs(point.x * Mathf.Cos(theta) + point.y * Mathf.Sin(theta) - rho);
    }

    // Compute the Mahalanobis distance between this line and the given one:
    public float ComputeNormDistance(Line other) {
        // Perform some renamings to match the paper description:
        Vector<float> Xl = this.GetState();
        Vector<float> Xm = other.GetState();
        Matrix<float> Cl = this.covariance;
        Matrix<float> Cm = other.covariance;

        Vector<float> X = Xl - Xm;

        return (X.ToRowMatrix() * (Cl + Cm).Inverse() * X)[0];
    }

    // Supposing that this line belongs to the current model of the environment, use the given 
    // line (that was supposed to be matched with this one) to update this line position estimate,
    // covariance matrix and endpoints:
    public void UpdateLineUsingMatching(Line other) {
        // Perform some renamings to match the paper description:
        Vector<float> Xm = this.GetState();
        Vector<float> Xl = other.GetState();
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

    public Vector<float> GetState() {
        return V.DenseOfArray(new float[] { rho, theta });
    }
}
