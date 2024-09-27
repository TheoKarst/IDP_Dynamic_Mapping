using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Class used to represent lines primitives for the mapping using geometric primitives
/// </summary>
public class DynamicLine {
    
    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<double> M = Matrix<double>.Build;
    private static VectorBuilder<double> V = Vector<double>.Build;

    public struct LineState {
        public float rho;           // Radius of the line
        public float theta;         // Angle of the line
        public float dRho;          // Derivative of rho
        public float dTheta;        // Derivative of theta

        public LineState(float rho, float theta) {
            this.rho = rho;
            this.theta = theta;
            this.dRho = 0;
            this.dTheta = 0;
        }
    }

    // Color of the line (for drawing):
    public Color lineColor = Color.red;

    private LineState state;
    private Matrix<double> covariance;

    // Endpoints of the line:
    public Vector2 beginPoint, endPoint;

    // Use a BoolAxis to represent which parts of the line are valid, according to the current
    // observations from the LIDAR:
    private BoolAxis lineValidity = new BoolAxis(false);

    // Save the state of the line when we called ResetLineValidity() for the last time:
    private Vector2 lineValidityBegin;      // beginPoint of the line
    private Vector2 lineValidityDelta;      // endPoint - beginPoint
    private Vector2 lineValidityU;          // lineValidityDelta / lineValidityDelta.magSq

    // The minimum and maximum projection values of all the lines matched to this one,
    // along lineValidityU:
    private float lineValidityMinP, lineValidityMaxP;

    // If the line is considered as static:
    private bool _isStatic = false;
    private int lastMatchDynamic = 0;   // Match count the last time the line was considered as dynamic
    private int _matchCount = 0;        // Number of times this line was matched with an observation
    private bool _forceDelete = false;  // Flag used to force the deletion of a line

    public bool forceDelete { get => _forceDelete; }

    // For debugging and log purposes:
    private int _id;
    private static int _instantiatedLines = 0;

    public int id { get => _id; }
    public int matchCount { get => _matchCount; }
    public static int instantiatedLines {  get => _instantiatedLines; }

    public bool isStatic { get => _isStatic; }

    /// <summary>
    /// Instantiates a dynamic line to represent aligned parts of the environment
    /// </summary>
    /// <param name="rho">Distance of the line from the origin in meters</param>
    /// <param name="theta">Angle of the line in radians</param>
    /// <param name="covariance">2x2 covariance matrix of rho and theta</param>
    /// <param name="beginPoint">Begin position of the line</param>
    /// <param name="endPoint">End position of the line</param>
    public DynamicLine(float rho, float theta, Matrix<double> covariance, Vector2 beginPoint, Vector2 endPoint) {
        this.state = new LineState(rho, theta);

        // When building a new line, we cannot compute the covariance of der_rho and der_theta
        // because there is no way to estimate the speed of a line with one observation.
        // Even if the covariance matrix will be correctly updated later by the Kalman Filter,
        // the initialisation is still a problem...
        this.covariance = M.DenseOfArray(new double[,] {
            { covariance[0,0], covariance[0,1], 0, 0 },
            { covariance[1,0], covariance[1,1], 0, 0 },
            {      0         ,      0         , 0, 0 },
            {      0         ,      0         , 0, 0 },
        });

        this.beginPoint = beginPoint;
        this.endPoint = endPoint;

        CheckState();

        // Debug:
        _id = _instantiatedLines++;
    }

    /// <summary>
    /// Instantiates a copy of the given line, with redefined endpoints
    /// </summary>
    /// <param name="line">The line to copy</param>
    /// <param name="beginPoint">The new begin point of the line</param>
    /// <param name="endPoint">The new end point of the line</param>
    public DynamicLine(DynamicLine line, Vector2 beginPoint, Vector2 endPoint) {
        this.lineColor = line.lineColor;
        this.state = line.state;
        this.covariance = line.covariance;
        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
        this._matchCount = line.matchCount;
        this.lastMatchDynamic = line.lastMatchDynamic;
        this._isStatic = line._isStatic;

        CheckState();

        // Debug:
        _id = _instantiatedLines++;
    }

    /// <summary>
    /// Draws the line in the scene using Unity Gizmos
    /// </summary>
    /// <param name="height">Position of the line along the Z-axis</param>
    /// <param name="drawSpeedEstimate">If true, draws an </param>
    public void DrawGizmos(float height, bool drawSpeedEstimate) {
        Color color = _isStatic ? Color.black : lineColor;

        Vector3 p1 = Utils.To3D(beginPoint, height);
        Vector3 p2 = Utils.To3D(endPoint, height);
        Handles.DrawBezier(p1, p2, p1, p2, color, null, 4);


        // Draw an arrow representing the estimated speed of the line:
        if (drawSpeedEstimate) {
            // Compute the speed of the center of the line:
            Vector2 center = (beginPoint + endPoint) / 2;
            Vector2 speed = VelocityOfPoint(center.x, center.y);

            p1 = Utils.To3D(center, height);
            p2 = Utils.To3D(center + speed, height);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(p1, p2);
        }
    }

    /// <summary>
    /// From the previous line estimate and the elapsed time since the last update,
    /// predict where the line should be now.
    /// </summary>
    /// <param name="deltaTime">Elapsed time in seconds since the last update</param>
    /// <param name="Q">Process noise error (4x4 diagonal matrix)</param>
    /// <param name="friction">Friction applied to the speed of the line (between 0 and 1)</param>
    public void PredictState(float deltaTime, Matrix<double> Q, float friction) {
        // If the line is static, there is nothing to do:
        if (_isStatic)
            return;

        // The factor by which the velocity is multiplied with at each frame:
        float a = 1 - friction;

        // For the state prediction, we assume the line speed stayed the same,
        // and we use Euler integration step to update rho and theta:
        state.rho += deltaTime * state.dRho;
        state.theta += deltaTime * state.dTheta;
        state.dRho = a * state.dRho;
        state.dTheta = a * state.dTheta;

        // Then we compute the prediction for the line covariance matrix:
        // covariance = F.dot(covariance.dot(F.T)) + Q
        //
        // With:
        // F = [[1 0 deltaTime     0    ],
        //      [0 1     0     deltaTime],
        //      [0 0     a         0    ],
        //      [0 0     0         a    ]]

        // Renamings for simplicity:
        float h = deltaTime;
        float h2 = h * h;
        float a2 = a * a;
        Matrix<double> P = covariance;

        // First, we compute: P = F.dot(P.dot(F.T)).
        // The computation is hardcoded for efficiency, as F mostly contains zeros:
        double p00 = P[0, 0] + h * (P[2, 0] + P[0, 2]) + h2 * P[2, 2];
        double p01 = P[0, 1] + h * (P[2, 1] + P[0, 3]) + h2 * P[2, 3];
        double p10 = P[1, 0] + h * (P[3, 0] + P[1, 2]) + h2 * P[3, 2];
        double p11 = P[1, 1] + h * (P[3, 1] + P[1, 3]) + h2 * P[3, 3];

        double p02 = a * (P[0, 2] + h * P[2, 2]);
        double p03 = a * (P[0, 3] + h * P[2, 3]);
        double p12 = a * (P[1, 2] + h * P[3, 2]);
        double p13 = a * (P[1, 3] + h * P[3, 3]);

        double p20 = a * (P[2, 0] + h * P[2, 2]);
        double p21 = a * (P[2, 1] + h * P[2, 3]);
        double p30 = a * (P[3, 0] + h * P[3, 2]);
        double p31 = a * (P[3, 1] + h * P[3, 3]);

        double p22 = a2 * P[2, 2];
        double p23 = a2 * P[2, 3];
        double p32 = a2 * P[3, 2];
        double p33 = a2 * P[3, 3];

        // Then, we compute covariance = P + Q, where Q is a diagonal matrix:
        covariance[0, 0] = p00 + Q[0, 0];
        covariance[0, 1] = p01;
        covariance[0, 2] = p02;
        covariance[0, 3] = p03;

        covariance[1, 0] = p10;
        covariance[1, 1] = p11 + Q[1, 1];
        covariance[1, 2] = p12;
        covariance[1, 3] = p13;

        covariance[2, 0] = p20;
        covariance[2, 1] = p21;
        covariance[2, 2] = p22 + 0.1 * Mathf.Abs(state.dRho);   // + Q[2, 2];
        covariance[2, 3] = p23;

        covariance[3, 0] = p30;
        covariance[3, 1] = p31;
        covariance[3, 2] = p32;
        covariance[3, 3] = p33 + 0.1 * Mathf.Abs(state.dTheta); // + Q[3, 3];

        // We also need to update the endpoints of the line. To do so, we
        // simply project them on the new line:
        ProjectEndpoints();

        // Ensures that the new state is well defined:
        CheckState();
    }

    /// <summary>
    /// Replace the endpoints by their orthogonal projection along the line defined
    /// by the parameters(rho, theta)
    /// </summary>
    private void ProjectEndpoints() {

        float costheta = Mathf.Cos(state.theta);
        float sintheta = Mathf.Sin(state.theta);

        // Unit vector along the line:
        Vector2 u = new Vector2(-sintheta, costheta);

        // "Center" of the infinite line:
        Vector2 center = state.rho * new Vector2(costheta, sintheta);

        beginPoint = center + Vector2.Dot(beginPoint, u) * u;
        endPoint = center + Vector2.Dot(endPoint, u) * u;
    }

    /// <summary>
    /// Supposing that this line belongs to the current model of the environment,
    /// use the given line(that is supposed to be an observed line, matched with
    /// this one) to update the position estimate, covariance matrix and endpoints
    /// of this line.
    /// </summary>
    /// <param name="validityMargin">Used to extend the validity of an observed line</param>
    /// <param name="staticMaxRhoDerivative">Maximum value for the derivative of rho to consider
    /// the line static</param>
    /// <param name="staticMaxThetaDerivative">Maximum value for the derivative of theta to
    /// consider the line static</param>
    /// <param name="minMatchesToConsiderStatic">Minimum of frames to consider an immobile 
    /// line as static</param>
    public void UpdateLineUsingMatch(DynamicLine observation,
        float validityMargin, float staticMaxRhoDerivative,
        float staticMaxThetaDerivative, int minMatchesToConsiderStatic) {

        // First, update the state of this line from the observation, using
        // Kalman Filter:
        UpdateState(observation.state.rho, observation.state.theta,
            observation.covariance.SubMatrix(0, 2, 0, 2));

        // Check if the line should now be treated as dynamic or static:
        CheckDynamicStatus(staticMaxRhoDerivative, staticMaxThetaDerivative, minMatchesToConsiderStatic);

        // Then, since the observation is by definition valid, use it to
        // update which sections of this line are valid:
        UpdateLineValidity(observation.beginPoint, observation.endPoint, validityMargin);

        _matchCount++;
    }

    /// <summary>
    /// Use the given observation to update the line state 
    /// </summary>
    /// <param name="observationRho">Distance between the observed line and the center</param>
    /// <param name="observationTheta">Angle of the observed line in radians</param>
    /// <param name="observationCovariance">2x2 covariance matrix of the observation</param>
    private void UpdateState(float observationRho, float observationTheta, Matrix<double> observationCovariance) {
        Vector<double> innovation = V.DenseOfArray(new double[] {
            observationRho - state.rho,
            Utils.LineSubstractAngleRadians(observationTheta, state.theta)
        });

        // Compute the innovation covariance:
        // S = H.dot(covariance.dot(H.T)) + R
        //
        // With:
        // H = [[1 0 0 0],      R = observationCovariance
        //      [0 1 0 0]]

        Matrix<double> S = covariance.SubMatrix(0, 2, 0, 2) + observationCovariance;

        // Compute the optimal Kalman Gain:
        // K = covariance.dot(H.T).dot(inverse(S))
        Matrix<double> K = covariance.SubMatrix(0, covariance.RowCount, 0, 2) * S.Inverse();

        // Compute the updated state estimate:
        Vector<double> deltaState = K * innovation;
        state.rho += (float) deltaState[0];
        state.theta += (float) deltaState[1];
        state.dRho += (float) deltaState[2];
        state.dTheta += (float) deltaState[3];

        // Compute the updated state covariance:
        // covariance = covariance - K * H * covariance:
        covariance -= K * covariance.SubMatrix(0, 2, 0, covariance.ColumnCount);

        // Check if the new state is still well defined:
        CheckState();
    }

    /// <summary>
    /// Looks at the speed estimate of the line and updates if the line should be considered
    /// static or dynamic
    /// </summary>
    /// <param name="staticMaxRhoDerivative">Maximum value for the derivative of rho to consider
    /// the line static</param>
    /// <param name="staticMaxThetaDerivative">Maximum value for the derivative of theta to
    /// consider the line static</param>
    /// <param name="minMatchesToConsiderStatic">Minimum of frames to consider an
    /// immobile line as static</param>
    private void CheckDynamicStatus(float staticMaxRhoDerivative, float staticMaxThetaDerivative, int minMatchesToConsiderStatic) {
        // We check if the line is currently mooving:
        bool isMooving = Mathf.Abs(state.dRho) > staticMaxRhoDerivative
            || Mathf.Abs(state.dTheta) > staticMaxThetaDerivative;

        // If we are currently mooving, save the current match count, and make sure the line
        // is considered as dynamic:
        if(isMooving) {
            lastMatchDynamic = matchCount;
            _isStatic = false;
        }

        // Otherwise, if the line is not mooving for long enough, we can consider this line to be static:
        else if(!_isStatic && matchCount - lastMatchDynamic >= minMatchesToConsiderStatic) {
            _isStatic = true;
            state.dRho = 0;
            state.dTheta = 0;
        }

        // If the line is already static, we also make sure that the speed is zero:
        else if(_isStatic) {
            state.dRho = 0;
            state.dTheta = 0;
        }
    }

    /// <summary>
    /// Return if the given line (supposed to be part of the current world model) is a good candidate
    /// to be matched with this line. If this is the case, we will have to check the norm distance
    /// between the lines as a next step
    /// </summary>
    /// <param name="maxAngleDistance">Maximum angle (in radians) between the two lines</param>
    /// <param name="maxOrthogonalDistance">Maximum orthogonal distance of this line endpoints to the given model line</param>
    /// <param name="maxParallelDistance">Maximum distance of this line endpoints to the model line, along the model line</param>
    /// <returns>If the given model line fulfils the previous critera</returns>
    public bool IsMatchCandidate(DynamicLine modelLine, float maxAngleDistance, float maxOrthogonalDistance, float maxParallelDistance) {

        // If the angular difference between both lines is too big, the lines cannot match:
        if (Utils.LineDeltaAngleRadians(state.theta, modelLine.state.theta) > maxAngleDistance)
            return false;

        // If the orthogonal distance of the endpoints of this line
        // from the model line are too big, the lines cannot match:
        if (modelLine.DistanceOf(beginPoint) > maxOrthogonalDistance)
            return false;

        if (modelLine.DistanceOf(endPoint) > maxOrthogonalDistance)
            return false;

        // Finally, we check the minimum distance between the lines endpoints, along the model line:

        // Compute the unit vector along the model line:
        Vector2 u = new Vector2(-Mathf.Sin(modelLine.state.theta), Mathf.Cos(modelLine.state.theta));

        // Project all the endpoints along the model line:
        float p1 = Vector2.Dot(beginPoint, u);
        float p2 = Vector2.Dot(endPoint, u);
        float p3 = Vector2.Dot(modelLine.beginPoint, u);
        float p4 = Vector2.Dot(modelLine.endPoint, u);

        // Make sure p1 <= p2 and p3 <= p4:
        if (p1 > p2)
            (p1, p2) = (p2, p1);

        if (p3 > p4)
            (p3, p4) = (p4, p3);

        // The modelLine is a good match candidate only if the distance between
        // the intervals [p1, p2] and [p3, p4] is less than the given distance:
        return Mathf.Max(p3 - p2, p1 - p4) < maxParallelDistance;
    }

    /// <summary>
    /// Compute the Mahalanobis distance between this line (supposed to be an observed line)
    /// and the given one(supposed to be part of the model)
    /// </summary>
    public float NormDistanceFromModel(DynamicLine model) {
        // Since the speed of the observed lines is unknown (we cannot estimate the speed of
        // a line from a single frame), we only use rho and theta to compute the Norm Distance,
        // and ignore the speed of the line in the model:

        // Extract the covariance matrices of (rho, theta) for both lines:
        Matrix<double> Cl = this.covariance.SubMatrix(0, 2, 0, 2);
        Matrix<double> Cm = model.covariance.SubMatrix(0, 2, 0, 2);

        // We have to be cautious when substracting angles, to keep the result between -PI/2 and PI/2,
        // but X = Xl - Xm:
        Vector<double> X = V.DenseOfArray(new double[]{
            state.rho - model.state.rho,
            Utils.LineSubstractAngleRadians(state.theta, model.state.theta)
        });

        return (float)(X.ToRowMatrix() * (Cl + Cm).Inverse() * X)[0];
    }

    /// <summary>
    /// Set which parts of the line are valid or invalid, according to the current observations.
    /// </summary>
    /// <param name="startValid">If the beginPoint of this line should be marked as valid</param>
    /// <param name="changes">Sorted list of floats, between 0 (beginPoint, excluded) and 1 (endPoint, excluded),
    /// representing when we change from valid to invalid (or the opposite)</param>
    public void ResetLineValidity(bool startValid, List<float> changes) {
        // Save the current position of the line:
        lineValidityBegin = beginPoint;
        lineValidityDelta = endPoint - beginPoint;
        lineValidityU = lineValidityDelta / lineValidityDelta.sqrMagnitude;

        // Reset the minimum and maximum projection along the lineValidityU:
        lineValidityMinP = 0;
        lineValidityMaxP = 1;

        // By default, the line is marked as invalid, the beginPoint is valid, there is
        // a change at position 0 from invalid to valid:
        if (startValid)
            changes.Insert(0, 0);

        // Make sure the number of changes is a multiple of 2: since the line is by
        // default invalid, and has a finite part marked as valid, then the number
        // of changes should be even:
        if (changes.Count % 2 != 0)
            changes.Add(1);

        // The line is set to invalid by default, and this state changes for each point in the list:
        lineValidity.Reset(false, changes);
    }

    /// <summary>
    /// Updates the validity of the line, knowing that the section between match_begin and match_end
    /// should be considered valid
    /// </summary>
    /// <param name="matchBegin">Begin point of an observed line matched with this one</param>
    /// <param name="matchEnd">End point of an observed line matched with this one</param>
    /// <param name="margin">Parameter used to extend the validity of a line</param>
    private void UpdateLineValidity(Vector2 matchBegin, Vector2 matchEnd, float margin) {
        // Project the match along the initial line, in order to update which
        // sections of the line are valid:

        float pBegin = Vector2.Dot(matchBegin - lineValidityBegin, lineValidityU);
        float pEnd = Vector2.Dot(matchEnd - lineValidityBegin, lineValidityU);
        
        // Make sure pBegin <= pEnd:
        if (pBegin > pEnd)
            (pBegin, pEnd) = (pEnd, pBegin);

        // Update lineValidity min and max projections:
        if(pBegin < lineValidityMinP) lineValidityMinP = pBegin;
        if(pEnd > lineValidityMaxP) lineValidityMaxP = pEnd;

        // The line should be valid between pBegin-margin and pBegin+margin:
        lineValidity.SetValue(pBegin-margin, pEnd+margin, true);
    }

    /// <summary>
    /// Adds to the given list the parts of this line that are valid
    /// </summary>
    /// <param name="dest">The list on which new line segments will be added</param>
    /// <param name="minLineLength">Minimum length of a line</param>
    /// <param name="maxRhoErrorSq">Square of the maximum error estimate on the range of a line to keep it</param>
    /// <param name="maxThetaErrorSq">Square of the maximum error estimate on the angle of a line to keep it</param>
    /// <param name="initialisationSteps">Minimum number of steps where the line is kept alive, before checking the 
    /// two above thresholds</param>
    public void AddValidParts(List<DynamicLine> dest, float minLineLength, float maxRhoErrorSq, float maxThetaErrorSq, int initialisationSteps) {

        // If we need to delete this line, there is nothing to do:
        if (forceDelete)
            return;

        // If the error on the position estimate of the line is too high, then we have to remove it:
        if (_matchCount > initialisationSteps && (covariance[0, 0] > maxRhoErrorSq 
                                               || covariance[1, 1] > maxThetaErrorSq))
            return;

        ReadOnlyCollection<float> changes = lineValidity.GetSplits();

        // If there is only one valid section, resize this line and add it to dest:
        if(changes.Count == 2) {
            float min = Mathf.Max(changes[0], lineValidityMinP);
            float max = Mathf.Min(changes[1], lineValidityMaxP);

            if(max - min >= minLineLength) {
                beginPoint = lineValidityBegin + min * lineValidityDelta;
                endPoint = lineValidityBegin + max * lineValidityDelta;

                dest.Add(this);
            }
            return;
        }

        // Otherwise, create a new line for each long enough valid section:
        for(int i = 0; i < changes.Count; i+=2) {
            float min = Mathf.Max(changes[i], lineValidityMinP);
            float max = Mathf.Min(changes[i+1], lineValidityMaxP);

            if(max - min >= minLineLength) {
                Vector2 begin = lineValidityBegin + min * lineValidityDelta;
                Vector2 end = lineValidityBegin + max * lineValidityDelta;
                dest.Add(new DynamicLine(this, begin, end));
            }
        }
    }

    /// <summary>
    /// Return the orthogonal distance of the point from the line
    /// </summary>
    public float DistanceOf(Vector2 point) {
        return Mathf.Abs(point.x * Mathf.Cos(state.theta) + point.y * Mathf.Sin(state.theta) - state.rho);
    }

    /// <summary>
    /// If the forward direction is the vector from beginPoint to endPoint:
    /// <list type="bullet">
    /// <item>Returns -1 if the given point is on the left of the line</item>
    /// <item>Returns +1 if the given point is on the right of the line</item>
    /// <item>Returns 0 if the given point belongs to the line</item>
    /// </list>
    /// </summary>
    public int SideOfPoint(Vector2 point) {
        float dot = Vector2.Dot(Vector2.Perpendicular(beginPoint - endPoint), point - beginPoint);

        return dot == 0 ? 0 : dot > 0 ? 1 : -1;
    }

    /// <summary>
    /// Checks the state of the line, and make sure that we keep the following properties:
    /// <list type="bullet">
    /// <item>state.rho >= 0</item>
    /// <item>0 <= state.theta < 2*PI</item>
    /// </list>
    /// </summary>
    private void CheckState() {
        // If rho < 0, rotate the line to make r >= 0 again:
        if(state.rho < 0) {
            state.theta += Mathf.PI;
            state.rho = -state.rho;

            // Make sure to also update the speed to match this new representation !
            state.dRho = -state.dRho;
        }

        // Make sure theta is between 0 and 2*PI:
        state.theta = Mathf.Repeat(state.theta, 2*Mathf.PI);

        // This case probably never happens:
        if(state.theta == 2*Mathf.PI)
            state.theta = 0;
    }

    /// <summary>
    /// Computes the velocity of a point, supposing that this point belongs to the line
    /// </summary>
    /// <param name="x">World position of the point along the X-axis in meters</param>
    /// <param name="y">World position of the point along the Y-axis in meters</param>
    public Vector2 VelocityOfPoint(float x, float y) {
        // Perform some renamings for simplification:
        float rho = state.rho;
        float theta = state.theta;
        float dRho = state.dRho;
        float dTheta = state.dTheta;

        // Express the given point in the referential of the model line:
        float costheta = Mathf.Cos(theta), sintheta = Mathf.Sin(theta);
        Vector2 MP = new Vector2(x - rho * costheta, y - rho * sintheta);

        // Unit vectors orthogonal to the line, and along the line:
        Vector2 x1 = new Vector2(costheta, sintheta);       // Orthogonal
        Vector2 y1 = new Vector2(-sintheta, costheta);      // Along
        
        // MP = a.x1 + b.y1:
        float a = Vector2.Dot(MP, x1);
        float b = Vector2.Dot(MP, y1);

        // Compute the derivative of the point in the referential of the line:
        float der_along_x1 = dRho - b * dTheta;
        float der_along_y1 = (rho + a) * dTheta;

        // Express the result in the base reference:
        return new Vector2(
            der_along_x1 * costheta - der_along_y1 * sintheta,
            der_along_x1 * sintheta + der_along_y1 * costheta);
    }


    /// <summary>
    /// Returns the value t such that the point:
    /// I = line.begin_point + t * (line.end_point - line.begin_point)
    /// belongs to the line AB.This point is thus the intersection point
    /// between this line and the line AB
    /// </summary>
    public float IntersectDistance(Vector2 A, Vector2 B) {
        Vector2 AB = B - A;
        Vector2 CD = endPoint - beginPoint;

        float den = CD.x * AB.y - CD.y * AB.x;

        if (den == 0)
            return -1;

        Vector2 AC = beginPoint - A;

        return (AC.y * AB.x - AC.x * AB.y) / den;
    }

    /// <summary>
    /// Forces the deletion of the line
    /// </summary>
    public void ForceDelete() {
        _forceDelete = true;
    }

    // Used for log purposes:
    public LineState GetState() { return state; }
    public Matrix<double> GetCovariance() { return covariance; }

    public override string ToString() {
        float print_rho = Utils.Round(state.rho, 2);
        float print_theta = Utils.Round(Mathf.Rad2Deg * state.theta, 2);
        string print_d_rho = Utils.ToString(state.dRho);
        string print_d_theta = Utils.ToString(Mathf.Rad2Deg * state.dTheta);

        float print_cov_rho = Utils.Round(Mathf.Sqrt((float)covariance[0, 0]), 2);
        float print_cov_theta = Utils.Round(Mathf.Rad2Deg * Mathf.Sqrt((float)covariance[1, 1]), 2);
        string print_cov_d_rho = Utils.ToString(Mathf.Sqrt((float)covariance[2, 2]));
        string print_cov_d_theta = Utils.ToString(Mathf.Rad2Deg * Mathf.Sqrt((float)covariance[3, 3]));

        return "Dynamic Line: [" + beginPoint + "; " + endPoint + "]: " +
            "rho=" + print_rho + " ±" + print_cov_rho +
            "; theta=" + print_theta + "° ±" + print_cov_theta +
            "; dRho=" + print_d_rho + " ±" + print_cov_d_rho +
            "; dTheta=" + print_d_theta + "°/s ±" + print_cov_d_theta;
    }
}