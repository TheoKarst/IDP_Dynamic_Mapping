using MathNet.Numerics.LinearAlgebra;
using UnityEditor;
using UnityEngine;

public class Line {
    private Vector<float> state;
    private Matrix<float> covariance;

    private Vector2 beginPoint, endPoint;

    public Line(float rho, float theta, Matrix<float> covariance, Vector2 beginPoint, Vector2 endPoint) {
        this.state = Vector<float>.Build.DenseOfArray(new float[] { rho, theta });
        this.covariance = covariance;
        this.beginPoint = beginPoint;
        this.endPoint = endPoint;
    }

    public void DrawGizmos() {
        Vector3 p1 = new Vector3(beginPoint.x, 0.5f, beginPoint.y);
        Vector3 p2 = new Vector3(endPoint.x, 0.5f, endPoint.y);
        Handles.DrawBezier(p1, p2, p1, p2, Color.red, null, 4);
    }
}
