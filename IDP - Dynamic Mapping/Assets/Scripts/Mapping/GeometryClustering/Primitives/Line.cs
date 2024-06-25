using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Profiling;

public class DeprecatedLine : Primitive {
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

    // Use a BoolAxis to represent which parts of the line are valid, according to the current
    // observations from the LIDAR:
    private BoolAxis lineValidity = new BoolAxis(0, 1, true);
    private Vector2 lineValidityBegin, lineValidityEnd, lineValidityU;

    // Copy constructor:
    public DeprecatedLine(DeprecatedLine line) : this(line, line.beginPoint, line.endPoint) {}

    // Create a line with updated endpoints (the parameters and line covariance
    // will stay the same, no matter the endpoints):
    public DeprecatedLine(DeprecatedLine other, Vector2 beginPoint, Vector2 endPoint) {
        lineColor = other.lineColor;

        rho = other.rho;
        theta = other.theta;
        rhoP = other.rhoP;
        thetaP = other.thetaP;

        covariance = other.covariance;

        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
    }

    public DeprecatedLine(float rho, float theta, Matrix<double> covariance, Vector2 beginPoint, Vector2 endPoint) {
        this.rho = rho;
        this.theta = Mathf.Repeat(theta, 2*Mathf.PI);
        
        this.rhoP = this.thetaP = 0;

        this.covariance = covariance;

        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
    }

    public void DrawGizmos(float height) {
        Vector3 p1 = Utils.To3D(beginPoint, height);
        Vector3 p2 = Utils.To3D(endPoint, height);
        Handles.DrawBezier(p1, p2, p1, p2, lineColor, null, 4);
    }

    // Return if the given line (supposed to be part of the current world model) is a good candidate
    // to be matched with this line. If this is the case, we will have to check the norm distance
    // between the lines as x next step:
    public bool IsMatchCandidate(DeprecatedLine modelLine, float maxAngleDistance, float maxEndpointDistance) {
        Profiler.BeginSample("Is Match Candidate");

        // If the angular difference between both lines is too big, the lines cannot match:
        if (Mathf.PingPong(Mathf.Abs(theta - modelLine.theta), Mathf.PI/2) > maxAngleDistance) {
            Profiler.EndSample();
            return false;
        }

        // If the distance of the endpoints of this line to the 
        if (modelLine.DistanceFrom(beginPoint) > maxEndpointDistance) {
            Profiler.EndSample();
            return false;
        }

        if (modelLine.DistanceFrom(endPoint) > maxEndpointDistance) {
            Profiler.EndSample();
            return false;
        }

        // If both lines are close enough, then the other line should ba x good candidate for
        // the matching:
        Profiler.EndSample();
        return true;
    }

    public string LogMatchCandidate(DeprecatedLine modelLine, float maxAngleDistance, float maxEndpointDistance) {
        float deltaAngle = Mathf.PingPong(Mathf.Abs(theta - modelLine.theta), Mathf.PI / 2);
        string d1 = Utils.ScientificNotation(Mathf.Rad2Deg * deltaAngle);
        string d2 = Utils.ScientificNotation(modelLine.DistanceFrom(beginPoint));
        string d3 = Utils.ScientificNotation(modelLine.DistanceFrom(endPoint));

        return "[MC: " + d1 + "°, " + d2 + ", " + d3 + "=>" + IsMatchCandidate(modelLine, maxAngleDistance, maxEndpointDistance) + "]";
    }

    public string LogNormDistance(DeprecatedLine other) {
        // Perform some renamings to match the paper description:
        Matrix<double> Cl = this.covariance;
        Matrix<double> Cm = other.covariance;

        // We have to be cautious when substracting angles, to keep the result between -PI/2 and PI/2,
        // but X = Xl - Xm:
        Vector<double> X = V.DenseOfArray(new double[]{
            rho - other.rho,
            Utils.LineSubstractAngleRadians(theta, other.theta)
        });

        float result = (float)(X.ToRowMatrix() * (Cl + Cm).Inverse() * X)[0];

        return "[Cl: " + Cl + "; Cm: " + Cm + "; X: " + X + " => result: " + result + "]";
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

    public float SignedDistanceFrom(Vector2 point) {
        return point.x * Mathf.Cos(theta) + point.y * Mathf.Sin(theta) - rho;
    }

    public float DistanceOf(Vector2 point) {
        return Vector2.Dot(Vector2.Perpendicular(endPoint - beginPoint), point - beginPoint);
    }

    // Compute the Mahalanobis distance between this line and the given one:
    public float ComputeNormDistance(DeprecatedLine other) {
        Profiler.BeginSample("Compute Norm Distance");

        // Perform some renamings to match the paper description:
        Matrix<double> Cl = this.covariance;
        Matrix<double> Cm = other.covariance;

        // We have to be cautious when substracting angles, to keep the result between -PI/2 and PI/2,
        // but X = Xl - Xm:
        Vector<double> X = V.DenseOfArray(new double[]{
            rho - other.rho,
            Utils.LineSubstractAngleRadians(theta, other.theta) 
        });

        float result = (float) (X.ToRowMatrix() * (Cl + Cm).Inverse() * X)[0];
        Profiler.EndSample();

        return result;
    }

    // Compute the distance between the centers of the two lines, along this line:
    public float ComputeCenterDistance(DeprecatedLine modelLine) {
        Vector2 thisCenter = (beginPoint + endPoint) / 2;
        Vector2 otherCenter = (modelLine.beginPoint + modelLine.endPoint) / 2;

        // Unit vector along the line:
        Vector2 u = new Vector2(-Mathf.Sin(theta), Mathf.Cos(theta));

        // Project the vector along u:
        return Mathf.Abs(Vector2.Dot(otherCenter - thisCenter, u));
    }

    // Supposing that this line belongs to the current model of the environment, use the given 
    // line (that is supposed to be matched with this one) to update the position estimate,
    // covariance matrix and endpoints of this line:
    public void UpdateLineUsingMatching(DeprecatedLine other) {
        // Perform some renamings to match the paper description:
        Vector<double> Xl = V.DenseOfArray(new double[] {other.rho, other.theta});
        Matrix<double> Cm = this.covariance;
        Matrix<double> Cl = other.covariance;

        // Use x static Kalman Filter to update this line covariance and state (rho, theta) estimate:
        Matrix<double> K = Cl * (Cl + Cm).Inverse();
        Vector<double> Xr = Xl + K * SubstractStates(this, other);
        Matrix<double> Cr = Cl - K * Cl;

        if (Cr[0, 0] < 0 || Cr[1, 1] < 0)
            Debug.LogError("Error when updating line covariance during matching.");

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

        // "Center" of the infinite line:
        Vector2 center = new Vector2(rho * costheta, rho * sintheta);

        // Unit vector along the line:
        Vector2 u = new Vector2(-sintheta, costheta);

        // Compute the projection of the given points along this line:
        float proj1 = Vector2.Dot(beginPoint - center, u);
        float proj2 = Vector2.Dot(endPoint - center, u);
        float proj3 = Vector2.Dot(other.beginPoint - center, u);
        float proj4 = Vector2.Dot(other.endPoint - center, u);

        float pMin = Mathf.Min(proj1, proj2, proj3, proj4);
        float pMax = Mathf.Max(proj1, proj2, proj3, proj4);

        // Debug.Log("Resize line: [" + proj1 + ", " + proj2 + "] => [" + pMin + ", " + pMax + "]");

        beginPoint = center + pMin * u;
        endPoint = center + pMax * u;

        // We also have to update which parts of the line are valid or not:
        UpdateValidity(other.beginPoint, other.endPoint);
    }

    // Return the length of the line, using its endpoints
    public float Length() {
        return Vector2.Distance(beginPoint, endPoint);
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

    private static Vector<double> SubstractStates(DeprecatedLine a, DeprecatedLine b) {
        // We have to be cautious when substracting angles, to keep the result between -PI/2 and PI/2:
        return V.DenseOfArray(new double[]{
            a.rho - b.rho,
            Utils.LineSubstractAngleRadians(a.theta, b.theta)
        });
    }

    // Set which parts of the line are valid or invalid, according to the current observations:
    public void UpdateValidity(List<float> changes, bool startValid) {
        lineValidity.Reset(0, 1, startValid, changes);
        lineValidityBegin = beginPoint;
        lineValidityEnd = endPoint;

        lineValidityU = endPoint - beginPoint;
        lineValidityU /= lineValidityU.sqrMagnitude;

        // Debug.Log("Reset validity [0, 1]: " + lineValidity);
    }

    private void UpdateValidity(Vector2 matchBegin, Vector2 matchEnd) {
        float pBegin = Vector2.Dot(matchBegin - lineValidityBegin, lineValidityU);
        float pEnd = Vector2.Dot(matchEnd - lineValidityBegin, lineValidityU);

        if (pBegin > pEnd)
            (pBegin, pEnd) = (pEnd, pBegin);

        // TEST:
        pBegin -= 0.1f;
        pEnd += 0.1f;

        lineValidity.SetValue(pBegin, pEnd, true);
        // Debug.Log("Update validity: [" + pBegin + ", " + pEnd + "]");
    }

    public void AddValidParts(List<DeprecatedLine> dest, float minLineLength) {
        // string logMsg = "Line validity: " + lineValidity + " => ";
        
        if (lineValidity.IsConstant()) {
            if (lineValidity.StartValue() == true) {
                dest.Add(this);
                // Debug.Log(logMsg += "Keep line");
            }
            // else {
            //     Debug.LogWarning("Deleting line: [" + beginPoint + "; " + endPoint + "]\n"
            //         + "Line validity: " + lineValidity);
            // }
            return;
        }

        float costheta = Mathf.Cos(theta), sintheta = Mathf.Sin(theta);
        Vector2 center = new Vector2(rho * costheta, rho * sintheta);
        Vector2 u = new Vector2(-sintheta, costheta);

        // bool DEBUG_DELETE_LINE = true;
        // logMsg += "Split line: ";
        foreach ((float min, float max) validZone in lineValidity.GetTrueZones()) {
            Vector2 p1 = lineValidityBegin + validZone.min * (lineValidityEnd - lineValidityBegin);
            Vector2 p2 = lineValidityBegin + validZone.max * (lineValidityEnd - lineValidityBegin);

            float proj1 = Vector2.Dot(p1 - center, u);
            float proj2 = Vector2.Dot(p2 - center, u);

            if (Mathf.Abs(proj2 - proj1) >= minLineLength) {
                dest.Add(new DeprecatedLine(this, center + proj1 * u, center + proj2 * u));
                // logMsg += "[" + validZone.min + "; " + validZone.max + "]; ";
                // DEBUG_DELETE_LINE = false;
            }
        }

        /*if(DEBUG_DELETE_LINE) {
            Debug.LogWarning(logMsg + " => DELETE !");
        }
        else
            Debug.Log(logMsg);*/
    }

    /////////////////////////////////////////////////////////////////////////////////////
    //// TODO: Remove the unnecessary line intersection computing functions...
    /////////////////////////////////////////////////////////////////////////////////////

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

    // Compute the intersection between two infinite lines, and return if the intersection exists:
    public static (bool, Vector2) LineIntersect(Vector2 A, Vector2 B, Vector2 C, Vector2 D) {
        Vector2 AB = B - A;
        Vector2 CD = D - C;

        float den = AB.x * CD.y - AB.y * CD.x;

        // If lines are parallel, there is no intersection:
        if (den == 0)
            return (false, Vector2.zero);

        Vector2 AC = C - A;
        float x = (AC.x * CD.y - AC.y * CD.x) / den;

        return (true, A + x * AB);
    }

    public float IntersectDistance(Vector2 A, Vector2 B) {
        Vector2 AB = B - A;
        Vector2 CD = endPoint - beginPoint;

        float den = CD.x * AB.y - CD.y * AB.x;

        if (den == 0)
            return -1;

        Vector2 AC = beginPoint - A;

        return (AC.y * AB.x - AC.x * AB.y) / den;
    }
}
