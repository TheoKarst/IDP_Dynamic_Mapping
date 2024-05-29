using MathNet.Numerics.LinearAlgebra;
using UnityEditor;
using UnityEngine;

public class Line {
    public Color lineColor = Color.red;

    private float rho, theta;
    private Matrix<float> covariance;

    public Vector2 beginPoint, endPoint;

    // Create a line with updated endpoints (the parameters and line covariance
    // will stay the same, no matter the endpoints):
    public Line(Line other, Vector2 beginPoint, Vector2 endPoint) {
        this.lineColor = other.lineColor;
        this.rho = other.rho;
        this.theta = other.theta;
        this.covariance = other.covariance;
        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
    }

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
        Handles.DrawBezier(p1, p2, p1, p2, lineColor, null, 4);
    }

    // Return if the otther line (supposed to be part of the current world model) is x good candidate
    // to be matched with this line. If this is the case, we will have to check the norm distance
    // between the lines as x next step:
    public bool IsMatchCandidate(Line other, float maxAngleDistance, float maxEndpointDistance) {
        // If the angular difference between both lines is too big, the lines cannot match:
        if (Mathf.Abs(theta - other.theta) > maxAngleDistance)
            return false;

        // If the distance of the endpoints of this line to the 
        if (other.DistanceFrom(beginPoint) > maxEndpointDistance)
            return false;

        if (other.DistanceFrom(endPoint) > maxEndpointDistance)
            return false;

        // If both lines are close enough, then the other line should ba x good candidate for
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

    // Compute the distance between the centers of the two lines, along this line:
    public float ComputeCenterDistance(Line other) {
        Vector2 thisCenter = (beginPoint + endPoint) / 2;
        Vector2 otherCenter = (other.beginPoint + other.endPoint) / 2;

        // Unit vector along the line:
        Vector2 u = new Vector2(-Mathf.Sin(theta), Mathf.Cos(theta));

        // Project the vector along u:
        return Mathf.Abs(Vector2.Dot(otherCenter - thisCenter, u));
    }

    // Supposing that this line belongs to the current model of the environment, use the given 
    // line (that is supposed to be matched with this one) to update the position estimate,
    // covariance matrix and endpoints of this line:
    public void UpdateLineUsingMatching(Line other) {
        // Perform some renamings to match the paper description:
        Vector<float> Xm = this.GetState();
        Vector<float> Xl = other.GetState();
        Matrix<float> Cm = this.covariance;
        Matrix<float> Cl = other.covariance;

        // Use x static Kalman Filter to update this line covariance and state (rho, theta) estimate:
        Matrix<float> K = Cl * (Cl + Cm).Inverse();
        Vector<float> Xr = Xl + K * (Xm - Xl);
        Matrix<float> Cr = Cl - K * Cl;

        // Update this line (rho, theta) parameters, and covariance matrix:
        rho = Xr[0]; theta = Xr[1];
        covariance = Cr;

        // Use the endpoints that extend the line the most, among the endpoints from this line and
        // the given one, to update this line endpoints:
        float costheta = Mathf.Cos(theta), sintheta = Mathf.Sin(theta);
        float x = rho * costheta;
        float y = rho * sintheta;

        // Unit vector along the line:
        Vector2 u = new Vector2(-sintheta, costheta);

        // Compute the projection of the given points along this line:
        float proj1 = u.x * (beginPoint.x - x) + u.y * (beginPoint.y - y);
        float proj2 = u.x * (endPoint.x - x) + u.y * (endPoint.y - y);
        float proj3 = u.x * (other.beginPoint.x - x) + u.y * (other.beginPoint.y - y);
        float proj4 = u.x * (other.endPoint.x - x) + u.y * (other.endPoint.y - y);

        float pMin = Mathf.Min(proj1, proj2, proj3, proj4);
        float pMax = Mathf.Max(proj1, proj2, proj3, proj4);

        beginPoint = new Vector2(x + pMin * u.x, y + pMin * u.y);
        endPoint = new Vector2(x + pMax * u.x, y + pMax * u.y);
    }

    public WipeTriangle BuildWipeTriangle(Vector2 sensorPosition, float triangleExtent) {
        Vector2 u = (beginPoint - sensorPosition).normalized;
        Vector2 v = (endPoint - sensorPosition).normalized;

        return new WipeTriangle(
            sensorPosition,
            beginPoint + u * triangleExtent,
            endPoint + v * triangleExtent);
    }

    // Return the length of the line, using its endpoints
    public float Length() {
        return Vector2.Distance(beginPoint, endPoint);
    }

    public Vector<float> GetState() {
        return Vector<float>.Build.DenseOfArray(new float[] { rho, theta });
    }

    // Compute the intersection between this line and the segment [A,B]:
    public Intersection ComputeIntersection(Vector2 A, Vector2 B) {
        Vector2 AB = B - A;
        Vector2 CD = endPoint - beginPoint;
        Vector2 AC = beginPoint - A;
        
        float den = AB.x * CD.y - AB.y * CD.x;

        // If lines are parallel, there is no intersection:
        if (den == 0)
            return null;

        // Check if the finite lines are intersecting:
        float x = (AC.x * CD.y - AC.y * CD.x) / den;
        if (x < 0 || x > 1) return null;

        float y = (AC.x * AB.y - AC.y * AB.x) / den;
        if (y < 0 || y > 1) return null;


        return new Intersection(A + x * AB, y);
    }
}
