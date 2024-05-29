using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class VehicleModel {
    // Create shortcuts for state, landmarks and observations dimensions:
    private const int STATE_DIM = VehicleState.DIMENSION;        // (x, y, phi)
    private const int LANDMARK_DIM = Landmark.DIMENSION;         // (x, y)
    private const int OBSERVATION_DIM = Observation.DIMENSION;   // (r, theta)

    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<float> M = Matrix<float>.Build;
    private static VectorBuilder<float> V = Vector<float>.Build;

    // Covariance matrices for the process noise errors (x, y, phi) and the observation errors (r, theta):
    public readonly Matrix<float> ProcessNoiseError;
    public readonly Matrix<float> ObservationError;

    // Parameters defining the dimensions of the model:
    private float a, b, L;

    public VehicleModel(float a, float b, float L) {
        this.a = a;
        this.b = b;
        this.L = L;

        // Model errors:
        float ePos = 0.4f;                  // Covariance of the position error
        float ePhi = 2 * Mathf.Deg2Rad;     // Covariance of the angular position error
        float eR = 0.4f;                    // Covariance of the measure distance error
        float eTheta = 2 * Mathf.Deg2Rad;   // Covariance of the measure angle error

        // Covariance matrix for the process noise errors (x, y, phi):
        this.ProcessNoiseError = M.Diagonal(new float[] {
            ePos * ePos,
            ePos * ePos,
            ePhi * ePhi });

        // Covariance matrix for the observation errors (r, theta):
        this.ObservationError = M.Diagonal(new float[] {
            eR * eR,
            eTheta * eTheta });
    }

    // Given the previous state estimate and the current inputs, compute the prediction 
    // of the new state estimate (Equation (10) of Dissanayake's paper):
    public VehicleState PredictCurrentState(VehicleState previous, ModelInputs inputs, float deltaT) {
        float x = previous.x + deltaT * inputs.V * Mathf.Cos(previous.phi);
        float y = previous.y + deltaT * inputs.V * Mathf.Sin(previous.phi);
        float phi = previous.phi + deltaT * inputs.V * Mathf.Tan(inputs.gamma) / L;
        
        return new VehicleState(x, y, phi);
    }

    // Given the previous state estimate and the current inputs, compute the Jacobian of the state
    // prediction, relatively to the state:
    public Matrix<float> StatePredictionJacobian(VehicleState previous, ModelInputs inputs, float deltaT) {
        float cosphi = Mathf.Cos(previous.phi), sinphi = Mathf.Sin(previous.phi);

        // See: PredictCurrentState()
        return M.DenseOfArray(new float[,] {
            {1, 0, -deltaT*inputs.V*sinphi},    // Derivative of   x / (previous.x, previous.y, previous.phi)
            {0, 1, deltaT*inputs.V*cosphi},     // Derivative of   y / (previous.x, previous.y, previous.phi)
            {0, 0, 1 } });                      // Derivative of phi / (previous.x, previous.y, previous.phi)
    }

    // Given the vehicle state estimate, and a global point in the map, return what observation should
    // correspond to the given point (this is used during Kalman Filter update).
    // See equations (11) and (37) of Dissanayake's paper:
    public Observation PredictObservation(VehicleState stateEstimate, float pointX, float pointY) {
        float cosphi = Mathf.Cos(stateEstimate.phi), sinphi = Mathf.Sin(stateEstimate.phi);

        float xr = stateEstimate.x + a * cosphi - b * sinphi;
        float yr = stateEstimate.y + a * sinphi + b * cosphi;

        float dX = pointX - xr;
        float dY = pointY - yr;

        float ri = Mathf.Sqrt(dX * dX + dY * dY);
        float thetai = Mathf.Atan2(dY, dX) - stateEstimate.phi;

        return new Observation(ri, thetai);
    }

    // From the vehicle state estimate and covariance, compute the position of the given observation in
    // global space, as well as the associated covariance matrix:
    public (Vector<float>, Matrix<float>) ComputeObservationPositionEstimate(
        VehicleState stateEstimate, Matrix<float> stateCovariance, Observation observation) {

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
                        + G * ObservationError.TransposeAndMultiply(G);

        return (Xp, Cp);
    }

    // Compute the Jacobian of the PredictObservation() function for a given landmark, with respect to the
    // state, and stack the result in dest matrix, at the given index:
    public void ComputeHi(VehicleState predictedState, Landmark landmark, int landmarkIndex, Matrix<float> dest, int index) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        
        float xi = landmark.x;
        float yi = landmark.y;
        float xr = predictedState.x + a * cosphi - b * sinphi;
        float yr = predictedState.y + a * sinphi + b * cosphi;

        float dX = xi - xr, dY = yi - yr;
        float dX2 = dX * dX, dY2 = dY * dY;
        float sqrt = Mathf.Sqrt(dX2 + dY2);

        // First row of the matrix:
        float A = -dX / sqrt;
        float B = -dY / sqrt;
        float C = (dX * (a * sinphi + b * cosphi) + dY * (b * sinphi - a * cosphi)) / sqrt;

        // Second row of the matrix:
        float D = dY / (dX2 + dY2);
        float E = -1 / (dX * (1 + dY2 / dX2));
        float F = ((-(a * cosphi - b * sinphi) * dX - dY * (a * sinphi + b * cosphi)) / (dX2 + dY2)) - 1;

        // Hv = Matrix(OBSERVATION_DIM, STATE_DIM):
        // (    dr/dx,      dr/dy,      dr/dphi)
        // (dtheta/dx,  dtheta/dy,  dtheta/dphi)
        Matrix<float> Hv = M.DenseOfArray(new float[,] { { A, B, C }, { D, E, F } });

        // Hpi = Matrix(OBSERVATION_DIM, LANDMARK_DIM):
        // (    dr/dxi,     dr/dyi)
        // (dtheta/dxi, dtheta/dyi)
        Matrix<float> Hpi = M.DenseOfArray(new float[,] { { -A, -B }, { -D, -E } });

        dest.SetSubMatrix(index, 0, Hv);
        dest.SetSubMatrix(index, STATE_DIM + landmarkIndex * LANDMARK_DIM, Hpi);
    }

    // From the vehicle state estimate, compute the world space position of the LIDAR:
    public Vector2 GetSensorPosition(VehicleState stateEstimate) {
        float cosphi = Mathf.Cos(stateEstimate.phi), sinphi = Mathf.Sin(stateEstimate.phi);

        float sensorX = stateEstimate.x + a * cosphi - b * sinphi;
        float sensorY = stateEstimate.y + a * sinphi + b * cosphi;

        return new Vector2(sensorX, sensorY);
    }
}
