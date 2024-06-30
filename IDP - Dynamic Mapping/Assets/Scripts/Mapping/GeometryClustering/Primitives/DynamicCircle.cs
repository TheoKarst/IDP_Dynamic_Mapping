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

    public void PredictState(float deltaTime) {
        // TODO: Implement this
    }

    public void UpdateState(Circle observation) {
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