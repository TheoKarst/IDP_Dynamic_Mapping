﻿using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using UnityEditor;
using UnityEngine;

public class DynamicLine : Primitive {

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

    public Color lineColor = Color.red;

    private LineState state;
    private Matrix<double> covariance;

    // Endpoints of the line, sorted so that if we turn in counter-clockwise
    // order around the origin, we will reach beginPoint first (see CheckState()):
    public Vector2 beginPoint, endPoint;

    // Use a BoolAxis to represent which parts of the line are valid, according to the current
    // observations from the LIDAR:
    private BoolAxis2 lineValidity = new BoolAxis2(false);

    // Position of the beginPoint, and unit vector from the beginPoint to the endPoint
    // of the line, when we called ResetLineValidity() for the last time:
    private Vector2 lineValidityBegin, lineValidityUnit;

    // The minimum and maximum projection values of all the lines matched to this one,
    // along the lineValidityUnit:
    private float lineValidityMinP, lineValidityMaxP;

    // To which line in the model this line is matched. This is used to have a speed estimate of all
    // the points from the current observation: the points are matched to current lines, and current
    // lines are matched with model lines, for which the velocity was correctly computed.
    // For each line already in the model, modelLine = this:
    private DynamicLine modelLine;

    public DynamicLine(float rho, float theta, Matrix<double> covariance, Vector2 beginPoint, Vector2 endPoint) {
        this.state = new LineState(rho, theta);

        // When building a new line, we cannot estimate the error on dRho and dTheta.
        // Thus, we initialise this part of the covariance matrix with zeros. The matrix
        // will be updated later with the Kalman Filter when we get more data about this line
        this.covariance = M.DenseOfArray(new double[,] {
            { covariance[0,0], covariance[0,1], 0, 0 },
            { covariance[1,0], covariance[1,1], 0, 0 },
            {      0         ,      0         , 0, 0 },
            {      0         ,      0         , 0, 0 },
        });

        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
        this.modelLine = this;
    }

    public DynamicLine(DynamicLine line, Vector2 beginPoint, Vector2 endPoint) {
        this.lineColor = line.lineColor;
        this.state = line.state;
        this.covariance = line.covariance;
        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
        this.modelLine = line.modelLine;
    }

    public void DrawGizmos(float height) {
        Vector3 p1 = Utils.To3D(beginPoint, height);
        Vector3 p2 = Utils.To3D(endPoint, height);
        Handles.DrawBezier(p1, p2, p1, p2, lineColor, null, 4);
    }

    // From the previous line estimate and the elapsed time since the last update,
    // predict where the line should be now.
    // Q is the process noise error (4x4 diagonal mtrix)
    public void PredictState(float deltaTime, Matrix<double> Q) {
        // For the state prediction, we assume the line speed stayed the same,
        // and we use Euler integration step to update rho and theta:
        state.rho += deltaTime * state.dRho;
        state.theta += deltaTime * state.dTheta;

        // Then we compute the prediction for the line covariance matrix:
        // covariance = F.dot(covariance.dot(F.T)) + Q
        //
        // With:
        // F = [[1 0 deltaTime     0    ],
        //      [0 1     0     deltaTime],
        //      [0 0     1         0    ],
        //      [0 0     0         1    ]]

        // Renamings for simplicity:
        float h = deltaTime;
        float h2 = h * h;
        Matrix<double> P = covariance;

        // First, we compute: P = F.dot(P.dot(F.T)).
        // The computation is hardcoded for efficiency, as F mostly contains zeros:
        double p00 = P[0, 0] + h * (P[2, 0] + P[0, 2]) + h2 * P[2, 2];
        double p01 = P[0, 1] + h * (P[2, 1] + P[0, 3]) + h2 * P[2, 3];
        double p02 = P[0, 2] + h * P[2, 2];
        double p03 = P[0, 3] + h * P[2, 3];

        double p10 = P[1, 0] + h * (P[3, 0] + P[1, 2]) + h2 * P[3, 2];
        double p11 = P[1, 1] + h * (P[3, 1] + P[1, 3]) + h2 * P[3, 3];
        double p12 = P[1, 2] + h * P[3, 2];
        double p13 = P[1, 3] + h * P[3, 3];

        double p20 = P[2, 0] + h * P[2, 2];
        double p21 = P[2, 1] + h * P[2, 3];
        // double p22 = P[2, 2];
        // double p23 = P[2, 3];

        double p30 = P[3, 0] + h * P[3, 2];
        double p31 = P[3, 1] + h * P[3, 3];
        // double p32 = P[3, 2];
        // double p33 = P[3, 3];

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
        covariance[2, 2] += Q[2, 2];
        // covariance[2, 3] = p23;

        covariance[3, 0] = p30;
        covariance[3, 1] = p31;
        // covariance[3, 2] = p32;
        covariance[3, 3] += Q[3, 3];

        // We also need to update the endpoints of the line. To do so, we
        // simply project them on the new line:
        beginPoint = Project(beginPoint);
        endPoint = Project(endPoint);

        // Check if the new state is well defined:
        CheckState();
    }

    // Use the given observation to update the line state.
    // R is the observation noise error (2x2 diagonal matrix)
    private void UpdateState(float observationRho, float observationTheta, Matrix<double> R) {
        Vector<double> innovation = V.DenseOfArray(new double[] {
            observationRho - state.rho,
            Utils.LineSubstractAngleRadians(observationTheta, state.theta)
        });

        // Compute the innovation covariance:
        // S = H.dot(covariance.dot(H.T)) + R
        //
        // With:
        // H = [[1 0 0 0],
        //      [0 1 0 0]]

        Matrix<double> S = M.DenseOfArray(new double[,] { 
            { covariance[0,0] + R[0,0], covariance[0,1] }, 
            { covariance[1,0], covariance[1,1] + R[1,1] } });

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
        if (modelLine.AbsDistanceOf(beginPoint) > maxOrthogonalDistance)
            return false;

        if (modelLine.AbsDistanceOf(endPoint) > maxOrthogonalDistance)
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

    // Compute the Mahalanobis distance between this line (supposed to be part of the current
    // observations), and the given one (supposed to belong to the world model):
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
    
    // Supposing that this line belongs to the current model of the environment,
    // use the given line (that is supposed to be matched with this one, and to
    // belong to the current observation of the environment) to update the position
    // estimate, covariance matrix and endpoints of this line:
    public void UpdateLineUsingMatch(DynamicLine observation, Matrix<double> lineObservationError, float validityMargin) {
        float TEST_THETA = state.theta;
        string prevState = this.ToString();
        
        // First, update the state of this line from the observation, using
        // Kalman Filter:
        UpdateState(observation.state.rho, observation.state.theta, lineObservationError);

        if (Utils.LineDeltaAngleRadians(TEST_THETA, state.theta) * Mathf.Rad2Deg > 10) {
            Debug.LogError(
                "Theta changed from: " + (Mathf.Rad2Deg * TEST_THETA) 
                + "° to " + (Mathf.Rad2Deg * state.theta) + "°"
                + "Model (before): " + prevState
                + "Match: " + observation
                + "Model (after): " + this);
        }
            

        // Then, since the observation is by definition valid, use it to
        // update which sections of this line are valid:
        UpdateLineValidity(observation.beginPoint, observation.endPoint, validityMargin);

        // Finally, match the observation with this line (supposed to be part of the model):
        observation.modelLine = this;
    }

    // Set which parts of the line are valid or invalid, according to the current observations.
    // startValid: If the beginPoint of this line should be marked as valid
    // changes: sorted list of floats, between 0 (beginPoint, excluded) and 1 (endPoint, excluded),
    //      representing when we change from valid to invalid (or the opposite)
    public void ResetLineValidity(bool startValid, List<float> changes) {
        // Save the current position of the line:
        lineValidityUnit = new Vector2(-Mathf.Sin(state.theta), Mathf.Cos(state.theta));
        lineValidityBegin = beginPoint;

        // Reset the minimum and maximum projection along the line:
        lineValidityMinP = 0;
        lineValidityMaxP = 1;

        // By default, the line is marked as invalid, the beginPoint is valid, there is
        // a change at position 0 from invalid to valid:
        if (startValid)
            changes.Insert(0, 0);

        // Make sure the number of changes is a multiple of 2: since the line is by
        // default invalid, and has a finite part marked as valid, tthen the number
        // of changes should be even:
        if (changes.Count % 2 != 0)
            changes.Add(1);

        // The line is set to invalid by default, and this state changes for each point in the list:
        lineValidity.Reset(false, changes);
    }

    private void UpdateLineValidity(Vector2 matchBegin, Vector2 matchEnd, float margin) {
        // Project the match along the initial line, in order to update which
        // sections of the line are valid:

        float pBegin = Vector2.Dot(matchBegin - lineValidityBegin, lineValidityUnit);
        float pEnd = Vector2.Dot(matchEnd - lineValidityBegin, lineValidityUnit);

        // Make sure pBegin <= pEnd:
        if (pBegin > pEnd)
            (pBegin, pEnd) = (pEnd, pBegin);

        // Update lineValidity min and max projections:
        if(pBegin < lineValidityMinP) lineValidityMinP = pBegin;
        if(pEnd > lineValidityMaxP) lineValidityMaxP = pEnd;

        // The line should be valid between pBegin-margin and pBegin+margin:
        lineValidity.SetValue(pBegin-margin, pEnd+margin, true);
    }

    // Add to the list the parts of this line that are valid:
    public void AddValidParts(List<DynamicLine> dest, float minLineLength) {
        ReadOnlyCollection<float> changes = lineValidity.GetSplits();

        // If there is only one valid section, resize this line and add it to dest:
        if(changes.Count == 2) {
            float min = Mathf.Max(changes[0], lineValidityMinP);
            float max = Mathf.Min(changes[1], lineValidityMaxP);

            if(max - min >= minLineLength) {
                beginPoint = lineValidityBegin + min * lineValidityUnit;
                endPoint = lineValidityBegin + max * lineValidityUnit;
                dest.Add(this);
            }
            return;
        }

        // Otherwise, create a new line for each long enough valid section:
        for(int i = 0; i < changes.Count; i+=2) {
            float min = Mathf.Max(changes[i], lineValidityMinP);
            float max = Mathf.Min(changes[i+1], lineValidityMaxP);

            if(max - min >= minLineLength) {
                Vector2 begin = lineValidityBegin + min * lineValidityUnit;
                Vector2 end = lineValidityBegin + max * lineValidityUnit;
                dest.Add(new DynamicLine(this, begin, end));
            }
        }
    }

    /// <summary>
    /// Return the orthogonal distance of the point from the line
    /// </summary>
    public float AbsDistanceOf(Vector2 point) {
        return Mathf.Abs(SignedDistanceOf(point));
    }

    /// <summary>
    /// Orthogonal distance of the point from the line, counted negative if the point is on the same
    /// side as the origin, and positive if the point is on the other side.
    /// If the forward direction is the vector from beginPoint to endPoint, the result will be positive
    /// if the given point is on the right of the line, and negative if the point is on the left
    /// </summary>
    public float SignedDistanceOf(Vector2 point) {
        return point.x * Mathf.Cos(state.theta) + point.y * Mathf.Sin(state.theta) - state.rho;
    }

    /// <summary>
    /// Compute the orthonal projection of the given point on the line
    /// </summary>
    private Vector2 Project(Vector2 point) {
        float costheta = Mathf.Cos(state.theta), sintheta = Mathf.Sin(state.theta);

        // Unit vector along the line:
        Vector2 u = new Vector2(-sintheta, costheta);

        // "Center" of the infinite line:
        Vector2 center = new Vector2(state.rho * costheta, state.rho * sintheta);

        return center + Vector2.Dot(point, u) * u;
    }

    // Check the state of the line, and make sure that we keep the following properties:
    // * state.rho >= 0
    // * 0 <= state.theta < 2*PI
    // * The triangle (origin, beginPoint, endPoint) is counter-clockwise
    private void CheckState() {
        // If rho < 0, rotate the line to make r >= 0 again:
        if(state.rho < 0) {
            state.theta += Mathf.PI;
            state.rho = -state.rho;
        }

        // Make sure theta is between 0 and 2*PI:
        state.theta = Mathf.Repeat(state.theta, 2*Mathf.PI);

        // This case probably never happens:
        if(state.theta == 2*Mathf.PI)
            state.theta = 0;

        // Make sure beginPoint and endPoint are in the expected order:
        if(Vector2.Dot(Vector2.Perpendicular(beginPoint), endPoint) < 0)
            (beginPoint, endPoint) = (endPoint, beginPoint);
    }

    public Vector2 VelocityOfPoint(float x, float y) {
        // Perform some renamings for simplification:
        float rho = modelLine.state.rho;
        float theta = modelLine.state.theta;
        float dRho = modelLine.state.dRho;
        float dTheta = modelLine.state.dTheta;

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

    public float IntersectDistance(Vector2 A, Vector2 B) {
        Vector2 AB = B - A;
        Vector2 CD = endPoint - beginPoint;

        float den = CD.x * AB.y - CD.y * AB.x;

        if (den == 0)
            return -1;

        Vector2 AC = beginPoint - A;

        return (AC.y * AB.x - AC.x * AB.y) / den;
    }

    public override string ToString() {
        float print_rho = Utils.Round(state.rho, 2);
        float print_theta = Utils.Round(Mathf.Rad2Deg * state.theta, 2);
        string print_d_rho = Utils.ScientificNotation(state.dRho);
        string print_d_theta = Utils.ScientificNotation(Mathf.Rad2Deg * state.dTheta);

        float print_cov_rho = Utils.Round(Mathf.Sqrt((float)covariance[0, 0]), 2);
        float print_cov_theta = Utils.Round(Mathf.Rad2Deg * Mathf.Sqrt((float)covariance[1, 1]), 2);
        string print_cov_d_rho = Utils.ScientificNotation(Mathf.Sqrt((float)covariance[2, 2]));
        string print_cov_d_theta = Utils.ScientificNotation(Mathf.Rad2Deg * Mathf.Sqrt((float)covariance[3, 3]));

        return "Dynamic Line: [" + beginPoint + "; " + endPoint + "]: " +
            "rho=" + print_rho + " ±" + print_cov_rho +
            "; theta=" + print_theta + "° ±" + print_cov_theta +
            "; dRho=" + print_d_rho + " ±" + print_cov_d_rho +
            "; dTheta=" + print_d_theta + "°/s ±" + print_cov_d_theta;
    }
}