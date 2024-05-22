using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class VehicleModel {

    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<float> M = Matrix<float>.Build;
    private static VectorBuilder<float> V = Vector<float>.Build;

    // Parameters defining the dimensions of the model:
    private float a, b, L;

    public VehicleModel(float a, float b, float L) {
        this.a = a;
        this.b = b;
        this.L = L;
    }

    // From the vehicle state estimate and covariance, compute the position of the given observation in
    // global space, as well as the associated covariance matrix:
    public (Vector<float>, Matrix<float>) computeObservationPositionEstimate(
        VehicleState stateEstimate, Matrix<float> stateCovariance, 
        Observation observation, Matrix<float> observationCovariance) {

        // Perform renamings for simplification:
        float x = stateEstimate.x, y = stateEstimate.y, phi = stateEstimate.phi;
        float r = observation.r, theta = observation.theta;

        // Compute some intermediate values:
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);
        float cosphi_theta = Mathf.Cos(phi + theta), sinphi_theta = Mathf.Sin(phi + theta);

        // 1. From the state estimate and the observation, compute the global position of the observation:
        // Xp = f(stateEstimate, observation):
        Vector<float> Xp = V.Dense(new float[] {
            x + a * cosphi - b * sinphi + r * cosphi_theta,
            y + a * sinphi + b * cosphi + r * sinphi_theta});

        // 2. Compute the Jacobian of f relatively to the vehicle state:
        Matrix<float> F = M.DenseOfArray(new float[,] {
            { 1, 0, -a * sinphi - b * cosphi - r * sinphi_theta},
            { 0, 1, a * cosphi - b * sinphi + r * cosphi_theta } });

        // 3. Compute the Jacobian of f relatively to the observation:
        Matrix<float> G = M.DenseOfArray(new float[,] {
            { cosphi_theta, -r * sinphi_theta },
            { sinphi_theta, r * cosphi_theta } });

        // 4. Now we can compute the covariance matrix associated to the observation position estimate:
        Matrix<float> Cp = F * stateCovariance.TransposeAndMultiply(F)
                        + G * observationCovariance.TransposeAndMultiply(G);

        return (Xp, Cp);
    }
}
