using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

/// <summary>
/// Class used to represent the model of a vehicle:
/// - Contains the equation for the state prediction
/// - Contains the equations to estimate the position of an observation from the vehicle
///   state, or to estimate which observation corresponds to a given position
/// - Contains the equations of the corresponding Jacobian matrices used in the Kalman Filter
/// </summary>

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

    // Setup of the different LIDARs attached to the vehicle:
    private LidarSetup[] lidarSetups;

    // Length of the vehicle:
    private float L;

    /// <summary>
    /// Instantiates a vehicle model
    /// </summary>
    /// <param name="lidarSetups">Setup of the LIDARs on the vehicle</param>
    /// <param name="L">Length of the vehicle in meters</param>
    /// <param name="processNoiseError">Process noise error of the state (3x3 matrix)</param>
    /// <param name="observationError">Observations error matrix (2x2 matrix)</param>
    public VehicleModel(LidarSetup[] lidarSetups, float L, Matrix<double> processNoiseError, Matrix<double> observationError) {
        this.lidarSetups = lidarSetups;
        this.L = L;

        this.ProcessNoiseError = processNoiseError;
        this.ObservationError = observationError;
    }

    /// <summary>
    /// Instantiates a vehicle model
    /// </summary>
    /// <param name="lidarSetups">Setup of the LIDARs on the vehicle</param>
    /// <param name="L">Length of the vehicle in meters</param>
    public VehicleModel(LidarSetup[] lidarSetups, float L) {
        this.lidarSetups = lidarSetups;
        this.L = L;

        float ePos = 0.1f;
        float ePhi = 3 * Mathf.Deg2Rad;
        float eR = 0.2f;
        float eTheta = 5 * Mathf.Deg2Rad;

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

    /// <summary>
    /// Given the previous state estimate and the current inputs, compute the prediction
    /// of the new state estimate (Equation (10) of Dissanayake's paper)
    /// </summary>
    public VehicleState PredictCurrentState(VehicleState previous, ModelInputs inputs, float deltaT) {
        float x = previous.x + deltaT * inputs.V * Mathf.Cos(previous.phi);
        float y = previous.y + deltaT * inputs.V * Mathf.Sin(previous.phi);
        float phi = previous.phi + deltaT * inputs.V * Mathf.Tan(inputs.gamma) / L;
        
        return new VehicleState(x, y, phi);
    }

    /// <summary>
    /// Given the previous state estimate and the current inputs, compute the Jacobian of the state
    /// prediction, relatively to the state
    /// </summary>
    public Matrix<double> StatePredictionJacobian(VehicleState previous, ModelInputs inputs, float deltaT) {
        float cosphi = Mathf.Cos(previous.phi), sinphi = Mathf.Sin(previous.phi);

        // See: PredictCurrentState()
        return M.DenseOfArray(new double[,] {
            {1, 0, -deltaT*inputs.V*sinphi},    // Derivative of   x / (previous.x, previous.y, previous.phi)
            {0, 1, deltaT*inputs.V*cosphi},     // Derivative of   y / (previous.x, previous.y, previous.phi)
            {0, 0, 1 } });                      // Derivative of phi / (previous.x, previous.y, previous.phi)
    }

    /// <summary>
    /// Given the vehicle state estimate, and a global point in the map, returns what observation (from the given
    /// LIDAR) should correspond to the given point. This is used during Kalman Filter update.
    /// See equations (11) and (37) of Dissanayake's paper
    /// </summary>
    public Observation PredictObservation(VehicleState stateEstimate, float pointX, float pointY, int lidarIndex) {
        // Get the local pose of the LIDAR whose observation we want to predict:
        Pose2D lidarPose = lidarSetups[lidarIndex].local_pose;

        float cosphi = Mathf.Cos(stateEstimate.phi), sinphi = Mathf.Sin(stateEstimate.phi);

        float xr = stateEstimate.x + lidarPose.x * cosphi - lidarPose.y * sinphi;
        float yr = stateEstimate.y + lidarPose.x * sinphi + lidarPose.y * cosphi;

        float dX = pointX - xr;
        float dY = pointY - yr;

        float ri = Mathf.Sqrt(dX * dX + dY * dY);
        float thetai = Mathf.Atan2(dY, dX) - stateEstimate.phi - lidarPose.angle;

        return new Observation(ri, thetai, lidarIndex, false);
    }

    /// <summary>
    /// From the vehicle state estimate, computes the position of the given observation in
    /// global space, without computing the corresponding covariance matrix
    /// </summary>
    public Vector2 ComputeObservationPositionEstimate(
        VehicleState stateEstimate, Observation observation) {

        // Get the local pose of the LIDAR which made the observation:
        Pose2D lidarPose = lidarSetups[observation.lidarIndex].local_pose;

        // Perform renamings for simplification:
        float a = lidarPose.x, b = lidarPose.y;
        float x = stateEstimate.x, y = stateEstimate.y, phi = stateEstimate.phi;
        float r = observation.r, theta = observation.theta + lidarPose.angle;

        // Compute some intermediate values:
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);
        float cosphi_theta = Mathf.Cos(phi + theta), sinphi_theta = Mathf.Sin(phi + theta);

        // From the state estimate and the observation, compute the global position of the observation:
        return new Vector2(x + a * cosphi - b * sinphi + r * cosphi_theta,
                           y + a * sinphi + b * cosphi + r * sinphi_theta);
    }

    /// <summary>
    /// From the vehicle state estimate and covariance, computes the position of the given observation in
    /// global space, as well as the associated covariance matrix
    /// </summary>
    public (Vector<double>, Matrix<double>) ComputeObservationPositionEstimate(
        VehicleState stateEstimate, Matrix<double> stateCovariance, Observation observation) {

        // Get the local pose of the LIDAR which made the observation:
        Pose2D lidarPose = lidarSetups[observation.lidarIndex].local_pose;

        // Perform renamings for simplification:
        float a = lidarPose.x, b = lidarPose.y;
        float x = stateEstimate.x, y = stateEstimate.y, phi = stateEstimate.phi;
        float r = observation.r, theta = observation.theta + lidarPose.angle;

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

    /// <summary>
    /// Computes the observation position estimate for a list of observations.
    /// The objective is to have a faster function
    /// </summary>
    public (Vector<double>[], Matrix<double>[]) ComputeObservationsPositionsEstimates(
        VehicleState stateEstimate, Matrix<double> stateCovariance, Observation[] observations, int lidarIndex) {

        // Get the local pose of the LIDAR which made the observations:
        Pose2D lidarPose = lidarSetups[lidarIndex].local_pose;

        // Arrays  to store the result:
        Vector<double>[] Xps = new Vector<double>[observations.Length];
        Matrix<double>[] Cps = new Matrix<double>[observations.Length];

        // Perform renamings for simplification:
        float a = lidarPose.x, b = lidarPose.y;
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
            Observation observation = observations[i];

            // If the observation is out of range, ignore it:
            if (observation.outOfRange)
                continue;

            float cosphi_theta = Mathf.Cos(phi + observation.theta + lidarPose.angle);
            float sinphi_theta = Mathf.Sin(phi + observation.theta + lidarPose.angle);

            float r_cosphi_theta = observation.r * cosphi_theta;
            float r_sinphi_theta = observation.r * sinphi_theta;

            // 1. From the state estimate and the observation, compute the global position of the observation:
            Xps[i] = V.Dense(new double[] {
                sensorX + r_cosphi_theta,
                sensorY + r_sinphi_theta 
            });

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

    /// <summary>
    /// Computes the Jacobian of the PredictObservation() function for a given landmark, with respect to the
    /// state, and stack the result in dest matrix, at the given index
    /// </summary>
    public void ComputeHi(VehicleState predictedState, Landmark landmark, int landmarkIndex, Matrix<double> dest, int index, int lidarIndex) {
        // Get the local pose of the LIDAR we want to use:
        Pose2D lidarPose = lidarSetups[lidarIndex].local_pose;
        float a = lidarPose.x, b = lidarPose.y;

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

    /// <summary>
    /// From the vehicle state estimate, computes the world space pose of the LIDAR
    /// </summary>
    public Pose2D GetWorldSensorPose(VehicleState stateEstimate, int lidarIndex) {
        // First, get the local pose of the LIDAR:
        Pose2D localPose = lidarSetups[lidarIndex].local_pose;

        float cosphi = Mathf.Cos(stateEstimate.phi), sinphi = Mathf.Sin(stateEstimate.phi);

        float a = localPose.x, b = localPose.y;
        float sensorX = stateEstimate.x + a * cosphi - b * sinphi;
        float sensorY = stateEstimate.y + a * sinphi + b * cosphi;

        return new Pose2D(sensorX, sensorY, stateEstimate.phi + localPose.angle);
    }

    /// <summary>
    /// Updates the local pose of a LIDAR on the vehicle (this is used by the DataloaderRobot)
    /// </summary>
    public void UpdateLidarPose(int lidarIndex, Pose2D newPose) {
        lidarSetups[lidarIndex].local_pose = newPose;
    }

    /// <summary>
    /// Returns the setup of the LIDAR with the given index
    /// </summary>
    public LidarSetup GetLidarSetup(int lidarIndex) {
        return lidarSetups[lidarIndex];
    }
}
