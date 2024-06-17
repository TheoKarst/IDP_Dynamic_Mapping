using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class VehicleModel {
    // Create shortcuts for state, landmarks and observations dimensions:
    private const int STATE_DIM = VehicleState.DIMENSION;        // (x, y, phi)
    private const int LANDMARK_DIM = Landmark.DIMENSION;         // (x, y)
    private const int OBSERVATION_DIM = Observation.DIMENSION;   // (r, theta)

    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<double> M = Matrix<double>.Build;
    private static VectorBuilder<double> V = Vector<double>.Build;

    // Covariance matrices for the process noise errors (x, y, phi) and the observation errors (r, theta):
    public readonly Matrix<double> ProcessNoiseError;
    public readonly Matrix<double> ObservationError;

    // Parameters defining the dimensions of the model:
    private float a, b, L;

    public VehicleModel(float a, float b, float L, Matrix<double> processNoiseError, Matrix<double> observationError) {
        this.a = a;
        this.b = b;
        this.L = L;

        this.ProcessNoiseError = processNoiseError;
        this.ObservationError = observationError;
    }

    public VehicleModel(float a, float b, float L, float maxSpeed, float maxSteering, float deltaTime) {
        this.a = a;
        this.b = b;
        this.L = L;

        // Covariance of the error on the linear and angular position of the robot:
        float ePos = 2 * deltaTime * maxSpeed;
        float ePhi = 2 * deltaTime * maxSpeed * Mathf.Tan(maxSteering) / L;
        
        // Covariance of the measure distance and angle of the LIDAR;
        float eR = 0.2f;
        float eTheta = 5 * Mathf.Deg2Rad;

        Debug.Log("Instantiating a robot model. ePos=" + ePos + ", ePhi=" + Mathf.Rad2Deg * ePhi
            + "°, eR=" + eR + ", eTheta=" + Mathf.Rad2Deg * eTheta + "°");

        // Covariance matrix for the process noise errors (x, y, phi):
        this.ProcessNoiseError = M.Diagonal(new double[] {
            ePos * ePos,
            ePos * ePos,
            ePhi * ePhi });

        // Covariance matrix for the observation errors (r, theta):
        this.ObservationError = M.Diagonal(new double[] {
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
    public Matrix<double> StatePredictionJacobian(VehicleState previous, ModelInputs inputs, float deltaT) {
        float cosphi = Mathf.Cos(previous.phi), sinphi = Mathf.Sin(previous.phi);

        // See: PredictCurrentState()
        return M.DenseOfArray(new double[,] {
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

    // From the vehicle state estimate, compute the position of the given observation in
    // global space, without computing the corresponding covariance matrix:
    public Vector2 ComputeObservationPositionEstimate(
        VehicleState stateEstimate, Observation observation) {

        // Perform renamings for simplification:
        float x = stateEstimate.x, y = stateEstimate.y, phi = stateEstimate.phi;
        float r = observation.r, theta = observation.theta;

        // Compute some intermediate values:
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);
        float cosphi_theta = Mathf.Cos(phi + theta), sinphi_theta = Mathf.Sin(phi + theta);

        // From the state estimate and the observation, compute the global position of the observation:
        return new Vector2(x + a * cosphi - b * sinphi + r * cosphi_theta,
                           y + a * sinphi + b * cosphi + r * sinphi_theta);
    }

    // From the vehicle state estimate and covariance, compute the position of the given observation in
    // global space, as well as the associated covariance matrix:
    public (Vector<double>, Matrix<double>) ComputeObservationPositionEstimate(
        VehicleState stateEstimate, Matrix<double> stateCovariance, Observation observation) {

        // Perform renamings for simplification:
        float x = stateEstimate.x, y = stateEstimate.y, phi = stateEstimate.phi;
        float r = observation.r, theta = observation.theta;

        // Compute some intermediate values:
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);
        float cosphi_theta = Mathf.Cos(phi + theta), sinphi_theta = Mathf.Sin(phi + theta);

        // 1. From the state estimate and the observation, compute the global position of the observation:
        // Xp = f(stateEstimate, observation):
        Vector<double> Xp = V.Dense(new double[] {
            x + a * cosphi - b * sinphi + r * cosphi_theta,
            y + a * sinphi + b * cosphi + r * sinphi_theta});

        // 2. Compute the Jacobian of f relatively to the vehicle state:
        Matrix<double> F = M.DenseOfArray(new double[,] {
            { 1, 0, -a * sinphi - b * cosphi - r * sinphi_theta},
            { 0, 1, a * cosphi - b * sinphi + r * cosphi_theta } });

        // 3. Compute the Jacobian of f relatively to the observation:
        Matrix<double> G = M.DenseOfArray(new double[,] {
            { cosphi_theta, -r * sinphi_theta },
            { sinphi_theta, r * cosphi_theta } });

        // 4. Now we can compute the covariance matrix associated to the observation position estimate:
        Matrix<double> Cp = F * stateCovariance.TransposeAndMultiply(F)
                        + G * ObservationError.TransposeAndMultiply(G);

        return (Xp, Cp);
    }

    // TESTING: Compute the observation position estimate for a list of observations.
    // The objective is to have a faster function:
    public (Vector<double>[], Matrix<double>[]) ComputeObservationsPositionsEstimates(
        VehicleState stateEstimate, Matrix<double> stateCovariance, AugmentedObservation[] observations) {

        // Arrays  to store the result:
        Vector<double>[] Xps = new Vector<double>[observations.Length];
        Matrix<double>[] Cps = new Matrix<double>[observations.Length];

        // Perform renamings for simplification:
        float x = stateEstimate.x, y = stateEstimate.y, phi = stateEstimate.phi;

        // Compute some intermediate values:
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);
        
        float sensorX = x + a * cosphi - b * sinphi;
        float sensorY = y + a * sinphi + b * cosphi;
        float F02_tmp = -a * sinphi - b * cosphi;
        float F12_tmp = a * cosphi - b * sinphi;

        // 1. Init F (-1 means we will compute this in the loop):
        Matrix<double> F = M.DenseOfArray(new double[,] {
            { 1, 0, -1 },
            { 0, 1, -1 } });

        // 2. Init G (-1 means we will compute this in the loop):
        Matrix<double> G = M.DenseOfArray(new double[,] {
            { -1, -1 },
            { -1, -1 } });
        
        // Now for each observation, compute Xp and Cp:
        for(int i = 0; i < observations.Length; i++) {
            AugmentedObservation observation = observations[i];

            float cosphi_theta = Mathf.Cos(phi + observation.theta);
            float sinphi_theta = Mathf.Sin(phi + observation.theta);

            float r_cosphi_theta = observation.r * cosphi_theta;
            float r_sinphi_theta = observation.r * sinphi_theta;

            // 1. From the state estimate and the observation, compute the global position of the observation:
            Xps[i] = V.Dense(new double[] {
                sensorX + r_cosphi_theta,
                sensorY + r_sinphi_theta 
            });

            // If the observation was out of range, no need to compute Cp:
            if(observation.outOfRange) {
                Cps[i] = null;
                continue;
            }

            // 2. Compute the Jacobian of f relatively to the vehicle state:
            F[0, 2] = F02_tmp - r_sinphi_theta;
            F[1, 2] = F12_tmp + r_cosphi_theta;

            // 3. Compute the Jacobian of f relatively to the observation:
            G[0, 0] = cosphi_theta;
            G[0, 1] = -r_sinphi_theta;
            G[1, 0] = sinphi_theta;
            G[1, 1] = r_cosphi_theta;

            // 4. Now we can compute the covariance matrix associated to the observation position estimate:
            Cps[i] = F * stateCovariance.TransposeAndMultiply(F)
                   + G * ObservationError.TransposeAndMultiply(G);
        }

        return (Xps, Cps);
    }

    // Compute the Jacobian of the PredictObservation() function for a given landmark, with respect to the
    // state, and stack the result in dest matrix, at the given index:
    public void ComputeHi(VehicleState predictedState, Landmark landmark, int landmarkIndex, Matrix<double> dest, int index) {
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
        Matrix<double> Hv = M.DenseOfArray(new double[,] { { A, B, C }, { D, E, F } });

        // Hpi = Matrix(OBSERVATION_DIM, LANDMARK_DIM):
        // (    dr/dxi,     dr/dyi)
        // (dtheta/dxi, dtheta/dyi)
        Matrix<double> Hpi = M.DenseOfArray(new double[,] { { -A, -B }, { -D, -E } });

        dest.SetSubMatrix(index, 0, Hv);
        dest.SetSubMatrix(index, STATE_DIM + landmarkIndex * LANDMARK_DIM, Hpi);
    }

    // From the vehicle state estimate, compute the world space pose of the LIDAR:
    public Pose2D GetSensorPose(VehicleState stateEstimate) {
        float cosphi = Mathf.Cos(stateEstimate.phi), sinphi = Mathf.Sin(stateEstimate.phi);

        float sensorX = stateEstimate.x + a * cosphi - b * sinphi;
        float sensorY = stateEstimate.y + a * sinphi + b * cosphi;

        return new Pose2D(sensorX, sensorY, stateEstimate.phi);
    }
}
