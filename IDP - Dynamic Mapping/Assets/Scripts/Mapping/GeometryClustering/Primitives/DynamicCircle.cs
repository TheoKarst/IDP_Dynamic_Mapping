using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class DynamicCircle : Primitive {
    public Color circleColor = Color.red;

    public struct CircleState {
        public float x, y;          // Position of the circle
        public float dx, dy;        // Derivative of x and y

        public CircleState(float x, float y) {
            this.x = x;
            this.y = y;
            this.dx = 0;
            this.dy = 0;
        }
    }

    private CircleState state;
    private Matrix<double> covariance;

    // Radius of the circle:
    private float R;

    private bool _isValid = true;
    public bool isValid { get { return _isValid; } }


    public DynamicCircle(float x, float y, float xCovSq, float yCovSq) {
        state = new CircleState(x, y);
        covariance = Matrix<double>.Build.DenseOfArray(new double[,] {
            {xCovSq,   0   , 0, 0 },
            {  0   , yCovSq, 0, 0 },
            {  0   ,   0   , 0, 0 },
            {  0   ,   0   , 0, 0 },
        });

        this.R = Mathf.Sqrt(xCovSq + yCovSq);
    }

    public void DrawGizmos(float height) {
        Gizmos.color = circleColor;
        Gizmos.DrawSphere(Utils.To3D(state.x, state.y, height), R);
    }

    public void PredictState(float deltaTime, Matrix<double> Q, float friction) {

        // The factor by which the velocity is multiplied with at each frame:
        float a = 1 - friction;

        // For the state prediction, we assume the line speed stayed the same,
        // and we use Euler integration step to update rho and theta:
        state.x += deltaTime * state.dx;
        state.y += deltaTime * state.dy;
        state.dx = a * state.dx;
        state.dy = a * state.dy;

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
        covariance[2, 2] = p22 + 0.1 * Mathf.Abs(state.dx);   // + Q[2, 2];
        covariance[2, 3] = p23;

        covariance[3, 0] = p30;
        covariance[3, 1] = p31;
        covariance[3, 2] = p32;
        covariance[3, 3] = p33 + 0.1 * Mathf.Abs(state.dy); // + Q[3, 3];
    }

    public void UpdateState(DynamicCircle observation) {
        // TODO: Implement this
    }
    

    // All the points belonging to a circle have the same speed, which is the speed
    // of the center of the circle:
    public Vector2 VelocityOfPoint(float x, float y) {
        return new Vector2(state.dx, state.dy);
    }

    // Set if the circle is valid (it's valid if the center of the circle is outside the
    // current WipeShape:
    public void UpdateValidity(bool isValid) {
        this._isValid = isValid;
    }
}