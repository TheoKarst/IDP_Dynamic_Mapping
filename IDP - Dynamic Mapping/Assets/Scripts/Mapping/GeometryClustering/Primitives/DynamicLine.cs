using MathNet.Numerics.LinearAlgebra;
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
    }

    public Color lineColor = Color.red;

    private LineState state;
    private Matrix<double> covariance;

    public Vector2 beginPoint, endPoint;

    // Use a BoolAxis to represent which parts of the line are valid, according to the current
    // observations from the LIDAR:
    private BoolAxis2 lineValidity = new BoolAxis2(false);

    // Position of the endpoints of the line before computing line matching:
    private Vector2 lineValidityBegin, lineValidityEnd, lineValidityUnit;

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
    }

    // Use the given observation to update the line state.
    // R is the observation noise error (2x2 diagonal matrix)
    public void UpdateState(float observationRho, float observationTheta, Matrix<double> R) {
        // Compute the innovation (WARNING WITH innovationTheta !):
        Vector<double> innovation = V.DenseOfArray(new double[] {
            observationRho - state.rho,
            observationTheta - state.theta,
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

        // Compute the updated state covariance:
        // covariance = covariance - K * H * covariance:
        covariance -= K * covariance.SubMatrix(0, 2, 0, covariance.ColumnCount);
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
            Utils.SubstractLineAngleRadians(state.theta, model.state.theta)
        });

        return (float)(X.ToRowMatrix() * (Cl + Cm).Inverse() * X)[0];
    }

    // Return if the given line (supposed to be part of the current world model) is a good candidate
    // to be matched with this line. If this is the case, we will have to check the norm distance
    // between the lines as x next step:
    public bool IsMatchCandidate(DynamicLine modelLine, float maxAngleDistance, float maxEndpointDistance) {
        
        // If the angular difference between both lines is too big, the lines cannot match:
        if (Utils.LineDeltaAngleRadians(state.theta, modelLine.state.theta) > maxAngleDistance)
            return false;

        // If the distance between the endpoints of this line and the model line
        // are too big, the lines cannot match:
        if (modelLine.DistanceOf(beginPoint) > maxEndpointDistance)
            return false;

        if (modelLine.DistanceOf(endPoint) > maxEndpointDistance)
            return false;

        return true;
    }
    private void UpdateLineValidity(Vector2 matchBegin, Vector2 matchEnd) {
        // Project the match along the initial line, in order to update which
        // sections of the line are valid:

        float pBegin = Vector2.Dot(matchBegin - lineValidityBegin, lineValidityUnit);
        float pEnd = Vector2.Dot(matchEnd - lineValidityBegin, lineValidityUnit);

        if (pBegin < pEnd)
            lineValidity.SetValue(pBegin, pEnd, true);
        else
            lineValidity.SetValue(pEnd, pBegin, true);
    }

    public Vector2 VelocityOfPoint(float x, float y) {
        throw new System.NotImplementedException();
    }
}