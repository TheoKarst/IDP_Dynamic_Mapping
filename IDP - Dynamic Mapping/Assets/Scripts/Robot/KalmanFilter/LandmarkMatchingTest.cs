using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class LandmarkMatchingTest : MonoBehaviour
{
    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<double> M = Matrix<double>.Build;
    private static VectorBuilder<double> V = Vector<double>.Build;

    public Transform robot;
    public Transform landmark1, landmark2;
    public float robotErrorX = 1, robotErrorY = 1, robotErrorPhi = 1;
    public float Rr = 1, Rtheta = 1;

    private const float lidarA = 0.1f, lidarB = 0f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDrawGizmos() {
        if(robot == null || landmark1 == null || landmark2 == null)
            return;

        float xk = robot.position.x;
        float yk = robot.position.z;
        float phik = Mathf.Deg2Rad * (90 - robot.rotation.eulerAngles.y);

        Matrix<double> Pv = M.Diagonal(new double[] { 
            robotErrorX * robotErrorX, 
            robotErrorY * robotErrorY, 
            (Mathf.Deg2Rad * robotErrorPhi) * (Mathf.Deg2Rad * robotErrorPhi) });

        Matrix<double> R = M.Diagonal(new double[] { 
            Rr * Rr,
            (Mathf.Deg2Rad * Rtheta) * (Mathf.Deg2Rad * Rtheta) });

        Vector<double> pf1, pf2;
        Matrix<double> Pf1, Pf2;
        (pf1, Pf1) = positionEstimate(Pv, R, landmark1, xk, yk, phik);
        (pf2, Pf2) = positionEstimate(Pv, R, landmark2, xk, yk, phik);

        Gizmos.color = Color.red;
        Gizmos.DrawCube(new Vector3(xk, robot.position.y, yk),
                        new Vector3(robotErrorX, 0, robotErrorY));

        Gizmos.color = Color.yellow;
        Gizmos.DrawCube(new Vector3((float) pf1[0], landmark1.position.y, (float) pf1[1]),
                        new Vector3(Mathf.Sqrt((float) Pf1[0, 0]), 0, Mathf.Sqrt((float) Pf1[1, 1])));

        Gizmos.DrawCube(new Vector3((float) pf2[0], landmark2.position.y, (float) pf2[1]),
                        new Vector3(Mathf.Sqrt((float) Pf2[0, 0]), 0, Mathf.Sqrt((float) Pf2[1, 1])));

        Vector<double> X = pf1 - pf2;
        float distance = (float)(X.ToRowMatrix() * (Pf1 + Pf2).Inverse() * X)[0];

        Debug.Log("Mahalanobis distance between landmarks: " + distance);
    }

    private (Vector<double>, Matrix<double>) positionEstimate(Matrix<double> Pv, Matrix<double> R, Transform landmark, float xk, float yk, float phik) {
        float dX = landmark.position.x - xk;
        float dY = landmark.position.z - yk;
        float rf = Mathf.Sqrt(dX * dX + dY * dY);
        float thetaf = Mathf.Atan2(dY, dX) - phik;

        Vector<double> pf = g(xk, yk, phik, rf, thetaf);
        Matrix<double> gradGxyp = computeGradGxyp(phik, rf, thetaf);    // Matrix(LANDMARK_DIM, STATE_DIM)
        Matrix<double> gradGrt = computeGradGrt(phik, rf, thetaf);      // Matrix(LANDMARK_DIM, OBSERVATION_DIM)

        // Pf = Matrix(LANDMARK_DIM, LANDMARK_DIM):
        Matrix<double> Pf = gradGxyp * Pv.TransposeAndMultiply(gradGxyp)
                        + gradGrt * R.TransposeAndMultiply(gradGrt);

        return (pf, Pf);
    }

    private Vector<double> g(float x, float y, float phi, float rf, float thetaf) {
        float a = lidarA, b = lidarB;
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);

        return V.Dense(new double[] {
            x + a * cosphi - b * sinphi + rf * Mathf.Cos(phi + thetaf),
            y + a * sinphi + b * cosphi + rf * Mathf.Sin(phi + thetaf)});
    }

    // Compute the gradient of g, relatively to x, y and phi:
    private Matrix<double> computeGradGxyp(float phi, float rf, float thetaf) {
        float a = lidarA, b = lidarB;
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);

        float dGx_dphi = -a * sinphi - b * cosphi - rf * Mathf.Sin(phi + thetaf);
        float dGy_dphi = a * cosphi - b * sinphi + rf * Mathf.Cos(phi + thetaf);

        return M.DenseOfArray(new double[,] {
            { 1, 0, dGx_dphi},
            { 0, 1, dGy_dphi } });
    }

    // Compute the gradient of g, relatively to rf and thetaf:
    private Matrix<double> computeGradGrt(float phi, float rf, float thetaf) {
        float cosphi_thetaf = Mathf.Cos(phi + thetaf);
        float sinphi_thetaf = Mathf.Sin(phi + thetaf);

        return M.DenseOfArray(new double[,] {
            { cosphi_thetaf, -rf * sinphi_thetaf },
            { sinphi_thetaf, rf * cosphi_thetaf } });
    }
}
