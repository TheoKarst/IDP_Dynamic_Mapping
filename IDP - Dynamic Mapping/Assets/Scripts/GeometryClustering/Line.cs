using MathNet.Numerics.LinearAlgebra;
using UnityEditor;
using UnityEngine;

public class Line : Primitive {
    // Vector builder used as a shortcut for vector creation:
    private static VectorBuilder<double> V = Vector<double>.Build;

    public Color lineColor = Color.red;

    private float rho, theta;       // Parameters of the line
    private float rhoP, thetaP;     // Derivative of rho and theta (used to estimate the speed of the line)

    // We need double precision for covariance matrices, since the lines in the model usually have
    // a much higher precision than the current lines, so matching them using float matrices may produce
    // wrong results:
    private Matrix<double> covariance;

    public Vector2 beginPoint, endPoint;

    // Copy constructor:
    public Line(Line line) : this(line, line.beginPoint, line.endPoint) {}

    // Create a line with updated endpoints (the parameters and line covariance
    // will stay the same, no matter the endpoints):
    public Line(Line other, Vector2 beginPoint, Vector2 endPoint) {
        lineColor = other.lineColor;

        rho = other.rho;
        theta = other.theta;
        rhoP = other.rhoP;
        thetaP = other.thetaP;

        covariance = other.covariance;

        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
    }

    public Line(float rho, float theta, Matrix<double> covariance, Vector2 beginPoint, Vector2 endPoint) {
        this.rho = rho;
        this.theta = Mathf.Repeat(theta, 2*Mathf.PI);
        
        this.rhoP = this.thetaP = 0;

        this.covariance = covariance;

        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
    }

    

    public void DrawGizmos() {
        Vector3 p1 = new Vector3(beginPoint.x, 0.2f, beginPoint.y);
        Vector3 p2 = new Vector3(endPoint.x, 0.2f, endPoint.y);
        Handles.DrawBezier(p1, p2, p1, p2, lineColor, null, 4);
    }

    // Return if the other line (supposed to be part of the current world model) is a good candidate
    // to be matched with this line. If this is the case, we will have to check the norm distance
    // between the lines as x next step:
    public bool IsMatchCandidate(Line other, float maxAngleDistance, float maxEndpointDistance) {
        // If the angular difference between both lines is too big, the lines cannot match:
        if (Utils.DeltaAngleRadians(theta, other.theta) > maxAngleDistance)
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

    public string LogMatchCandidate(Line other, float maxAngleDistance, float maxEndpointDistance) {
        string d1 = Utils.ScientificNotation(Mathf.Rad2Deg * Utils.DeltaAngleRadians(theta, other.theta));
        string d2 = Utils.ScientificNotation(other.DistanceFrom(beginPoint));
        string d3 = Utils.ScientificNotation(other.DistanceFrom(endPoint));

        return "[MC: " + d1 + "°, " + d2 + ", " + d3 + "=>" + IsMatchCandidate(other, maxAngleDistance, maxEndpointDistance) + "]";
    }

    public string LogParams() {
        string print_rho = Utils.ScientificNotation(rho);
        float print_theta = Utils.Round(Mathf.Rad2Deg * theta, 1);
        string print_cov_rho = Utils.ScientificNotation(Mathf.Sqrt((float) covariance[0, 0]));
        string print_cov_theta = Utils.ScientificNotation(Mathf.Rad2Deg * Mathf.Sqrt((float) covariance[1, 1]));
        
        return "rho=" + print_rho + " ±" + print_cov_rho + 
            "; theta=" + print_theta + "° ±" + print_cov_theta;
    }

    // Return the distance between the line and the given point:
    public float DistanceFrom(Vector2 point) {
        return Mathf.Abs(point.x * Mathf.Cos(theta) + point.y * Mathf.Sin(theta) - rho);
    }

    // Compute the Mahalanobis distance between this line and the given one:
    public float ComputeNormDistance(Line other) {
        // Perform some renamings to match the paper description:
        Matrix<double> Cl = this.covariance;
        Matrix<double> Cm = other.covariance;

        // We have to be cautious when substracting angles, to keep the result between -PI/2 and PI/2,
        // but X = Xl - Xm:
        Vector<double> X = V.DenseOfArray(new double[]{
            rho - other.rho,
            Utils.SubstractAngleRadians(theta, other.theta) 
        });

        return (float) (X.ToRowMatrix() * (Cl + Cm).Inverse() * X)[0];
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
        Vector<double> Xl = V.DenseOfArray(new double[] {other.rho, other.theta});
        Matrix<double> Cm = this.covariance;
        Matrix<double> Cl = other.covariance;

        // Use x static Kalman Filter to update this line covariance and state (rho, theta) estimate:
        Matrix<double> K = Cl * (Cl + Cm).Inverse();
        Vector<double> Xr = Xl + K * SubstractStates(this, other);
        Matrix<double> Cr = Cl - K * Cl;

        string log = "Updating line using match: " + Cm[0,0] + "; " + Cl[0,0] + "; " + Cr[0,0] + "\n";
        log += "K:  [[" + K[0, 0] + "; " + K[0, 1] + "]; [" + K[1, 0] + "; " + K[1, 1] + "]]\n";
        log += "Cm: [[" + Cm[0, 0] + "; " + Cm[0, 1] + "]; [" + Cm[1, 0] + "; " + Cm[1, 1] + "]]\n";
        log += "Cl: [[" + Cl[0, 0] + "; " + Cl[0, 1] + "]; [" + Cl[1, 0] + "; " + Cl[1, 1] + "]]\n";
        log += "Cr: [[" + Cr[0, 0] + "; " + Cr[0, 1] + "]; [" + Cr[1, 0] + "; " + Cr[1, 1] + "]]";
        if (Cr[0, 0] < 0 || Cr[1,1] < 0)
            Debug.LogError(log);
        else
            Debug.Log(log);

        (float nextRho, float nextTheta) = ReformulateState((float) Xr[0], (float) Xr[1]);

        // Update the line speed estimate, using a simple exponential low pass filter:
        const float m = 0.95f;
        other.rhoP = rhoP = m * rhoP + (1 - m) * (nextRho - rho);
        other.thetaP = thetaP = m * thetaP + (1 - m) * (nextTheta - theta);

        // Update this line (rho, theta) parameters, and covariance matrix:
        rho = nextRho; theta = nextTheta;
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

    public Vector2 VelocityOfPoint(float x, float y) {
        // First express the given point in the referential of the line:
        float costheta = Mathf.Cos(theta), sintheta = Mathf.Sin(theta);
        Vector2 MP = new Vector2(x - rho * costheta, y - rho * sintheta);

        // Unit vectors orthogonal to the line, and along the line:
        Vector2 x1 = new Vector2(costheta, sintheta);       // Orthogonal
        Vector2 y1 = new Vector2(-sintheta, costheta);      // Along
        
        // MP = a.x1 + b.y1:
        float a = Vector2.Dot(MP, x1);
        float b = Vector2.Dot(MP, y1);

        // Compute the derivative of the point in the referential of the line:
        float der_along_x1 = rhoP - b * thetaP;
        float der_along_y1 = (rho + a) * thetaP;

        // Express the result in the base reference:
        return new Vector2(
            der_along_x1 * costheta - der_along_y1 * sintheta,
            der_along_x1 * sintheta + der_along_y1 * costheta);
    }

    // Ensures that all the lines are respecting the same representation, which is:
    // rho (distance):    between 0 and +infinity
    // theta (radians):   between 0 (included) and 2*PI (excluded)
    private static (float, float) ReformulateState(float rho, float theta) {
        // If rho is negative, rotate the line to make it positive:
        if (rho < 0) {
            rho = -rho;
            theta += Mathf.PI;
        }

        // Express theta in the range [0, 2*PI]:
        theta = Mathf.Repeat(theta, 2 * Mathf.PI);

        // This case probably never happens...
        if (theta == 2 * Mathf.PI)
            theta = 0;

        return (rho, theta);
    }

    private static Vector<double> SubstractStates(Line a, Line b) {
        // We have to be cautious when substracting angles, to keep the result between -PI/2 and PI/2:
        return V.DenseOfArray(new double[]{
            a.rho - b.rho,
            Utils.SubstractAngleRadians(a.theta, b.theta)
        });
    }
}
