using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilter {
    // Define some constants for the dimensions of the state, landmarks and observations:
    private const int STATE_DIM = 3;            // (x, y, phi)
    private const int LANDMARK_DIM = 2;         // (x, y)
    private const int OBSERVATION_DIM = 2;      // (r, theta)

    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<float> M = Matrix<float>.Build;
    private static VectorBuilder<float> V = Vector<float>.Build;

    
    // According to the appendix II, create two lists of landmarks:
    private List<Landmark> confirmedLandmarks = new List<Landmark>();
    private List<Landmark> potentialLandmarks = new List<Landmark>();

    // Covariance matrices for the process noise and the observations:
    private Matrix<float> Q, R;

    // Current state estimate, and state covariance estimate of the robot:
    private ModelState stateEstimate;                        // x_hat(k|k)
    private StateCovariance stateCovarianceEstimate;    // P(k|k)

    // Save the last time of an update to determine the elapsed time between two updates:
    private float lastTimeUpdate;

    // Local position of the lidar and length of the vehicle, as defined in Dissanayake's paper:
    private Lidar lidar;
    private float lidarA, lidarB, vehicleL;

    public KalmanFilter(ModelState startState, Lidar lidar, float vehicleL) {
        this.lidar = lidar;
        this.lidarA = lidar.getLidarA();
        this.lidarB = lidar.getLidarB();
        this.vehicleL = vehicleL;

        Q = M.DenseDiagonal(STATE_DIM, 0.1f);           // Covariance matrix for the process noise errors (x, y, phi)
        R = M.DenseDiagonal(OBSERVATION_DIM, 0.01f);     // Covariance matrix for the observation errors (r, theta)

        // Initialize the state estimate with the given start position of the robot:
        stateEstimate = startState;
        stateCovarianceEstimate = new StateCovariance(STATE_DIM, 0, LANDMARK_DIM);
    }

    // TESTING: Define the landmarks manually:
    public void initLandmarks(Transform[] positions) {
        confirmedLandmarks.Clear();
        potentialLandmarks.Clear();

        foreach (Transform landmark in positions) {
            float x = landmark.position.x;
            float y = landmark.position.z;

            confirmedLandmarks.Add(new Landmark(x, y));
        }

        stateCovarianceEstimate = new StateCovariance(STATE_DIM, positions.Length, LANDMARK_DIM);
    }

    // Used for debugging: show the error on the position of each landmark:
    public void resizeLandmarksUsingCovariance(Transform[] landmarks) {
        for(int i = 0; i < landmarks.Length; i++) {
            Matrix<float> cov = stateCovarianceEstimate.extractLandmarkCovariance(i);

            // The landmark has a probability of 95% to be in this region:
            float scaleX = 2 * Mathf.Sqrt(cov[0, 0]);
            float scaleY = 2 * Mathf.Sqrt(cov[1, 1]);

            landmarks[i].localScale = new Vector3(scaleX, 0.1f, scaleY);
        }
    }

    public void resizeStateUsingCovariance(Transform transform) {
        Matrix<float> Pvv = stateCovarianceEstimate.extractPvv();

        // The robot has a probability of 95% to be in this region:
        float scaleX = 2 * Mathf.Sqrt(Pvv[0, 0]);
        float scaleY = 2 * Mathf.Sqrt(Pvv[1, 1]);

        transform.localScale = new Vector3(scaleX, 0.1f, scaleY);
        transform.position = new Vector3(stateEstimate.x, 1, stateEstimate.y);
    }

    // Given an observation from the sensor of the robot, the inputs of the robot and the current time,
    // try to associate the observation with a known landmark and update the robot state using the
    // "Extended Kalman Filter":
    public void updateStateEstimate(Observation observation, ModelInputs inputs, float currentTime) {
        float deltaT = currentTime - lastTimeUpdate;
        lastTimeUpdate = currentTime;

        //// 1. Prediction: Predict the current state estimate from our previous estimate and the current inputs

        // Equation (10): From the previous state estimate and current inputs,
        // predict a new state estimate: x_hat(k+1|k)
        ModelState statePrediction = predictStateEstimate(stateEstimate, inputs, deltaT);
        // Debug.Log("1. Previous state: " + stateEstimate + ", Inputs: " + inputs + " => Predicted state: " + statePrediction);

        // Equation (12): From the previous state estimate and the current inputs, compute the new estimate of the
        // state covariance matrix: P(k+1|k)
        Matrix<float> Fv = computeFv(stateEstimate, inputs, deltaT);
        StateCovariance stateCovariancePrediction = stateCovarianceEstimate.predictStateEstimateCovariance(Fv, Q);

        // Use the landmark association algorithm defined in the Appendix II to associate our observation with
        // a possible landmark:
        int landmarkIndex = computeLandmarkAssociation(observation, statePrediction, stateCovariancePrediction);
        
        // If no candidate landmark was found for our observation, cancel the update procedure:
        if (landmarkIndex == -1)
            return;

        // Equation (11): From the state prediction, and the landmark associated with the observation,
        // predict what the observation should be: z_hat(k+1|k)
        Observation observationPrediction = predictObservation(statePrediction, landmarkIndex);

        //// 2. Observation: Compare the expected observation with the real observation:
        Vector<float> innovation = Observation.substract(observation, observationPrediction);

        lidar.DrawObservation(observation, Color.green);             // Target observation
        lidar.DrawObservation(observationPrediction, Color.red);     // Predicted observation
        // Debug.Log("3. Innovation: (" + innovation[0] + ", " + innovation[1] + ")");

        Matrix<float> Hi, Si, Wi;
        Hi = computeHi(statePrediction, landmarkIndex, confirmedLandmarks.Count);           // Equation (37)
        (Si, Wi) = stateCovariancePrediction.computeInnovationAndGainMatrices(Hi, R);       // Equations (14) and (17)

        //// 3. Update: Update the state estimate from our observation
        // Debug.Log("4. Updated prediction: " + (statePrediction + Wi * innovation));

        stateEstimate = statePrediction + Wi * innovation;                                      // Equation (15)
        stateCovarianceEstimate = stateCovariancePrediction - Wi * Si.TransposeAndMultiply(Wi); // Equation (16)
    }

    // From the observation from the LIDAR, the predicted state, the predicted state covariance estimate
    // and the previous landmarks, return the index of the landmark that is the most likely to correspond
    // to the observation. If no appropriate landmark is found, the set of potential landmarks is updated.
    // return -1 if there is no candidate for the landmark
    private int computeLandmarkAssociation(Observation observation, ModelState statePrediction, StateCovariance stateCovariancePrediction) {
        const float dmin = 1f;

        // Perform some renamings for readability:
        float xk = statePrediction.x, yk = statePrediction.y, phik = statePrediction.phi;
        float rf = observation.r, thetaf = observation.theta;

        // Covariance matrix of the vehicle location estimate, extracted from P(k|k):
        Matrix<float> Pv = stateCovarianceEstimate.extractPvv();       // Matrix(STATE_DIM, STATE_DIM)

        // Compute pf, the position of the landmark possibly responsible for this observation:
        Vector<float> pf = g(xk, yk, phik, rf, thetaf);                // Matrix(LANDMARK_DIM, 1)
        // Debug.Log("2. State: " + statePrediction + "; Observation: " + observation + " => pf = (" + pf[0] + ", " + pf[1] + ")");

        // Compute Pf, the covariance matrix of pf:
        Matrix<float> gradGxyp = computeGradGxyp(phik, rf, thetaf);    // Matrix(LANDMARK_DIM, STATE_DIM)
        Matrix<float> gradGrt = computeGradGrt(phik, rf, thetaf);      // Matrix(LANDMARK_DIM, OBSERVATION_DIM)

        // Pv = Matrix(STATE_DIM, STATE_DIM) and R = M(OBSERVATION_DIM, OBSERVATION_DIM)
        // So Pf = Matrix(LANDMARK_DIM, LANDMARK_DIM):
        Matrix<float> Pf = gradGxyp * Pv.TransposeAndMultiply(gradGxyp)
                        + gradGrt * R.TransposeAndMultiply(gradGrt);

        int acceptedLandmark = -1;
        bool rejectObservation = false;
        float minDistance = -1;
        for (int i = 0; i < confirmedLandmarks.Count; i++) {
            Vector<float> pi = confirmedLandmarks[i].position();
            Matrix<float> Pi = stateCovariancePrediction.extractLandmarkCovariance(i);

            Vector<float> X = pf - pi;
            float dfi = (float)(X.ToRowMatrix() * (Pf + Pi).Inverse() * X)[0];

            if(minDistance == -1 || dfi < minDistance)
                minDistance = dfi;

            // If dfi < dmin, then the current landmark is chosen as a candidate for the observation:
            if (dfi < dmin) {
                // If no other landmark was accepted before, then accept this landmark:
                if (acceptedLandmark == -1) acceptedLandmark = i;

                // Else if more than one landmark is accepted for this observation, reject the observation:
                else {
                    Debug.LogError("Observation rejected: more than one landmark corresponds to this observation !");
                    rejectObservation = true;
                    break;
                }
            }
        }

        Debug.Log("Min distance: " + minDistance);

        // If a confirmed landmark was accepted for the observation, use it to generate a new state estimate:
        if (!rejectObservation && acceptedLandmark != -1)
            return acceptedLandmark;

        // Else if the observation was not rejected, we have to check it against the set of potential landmarks:
        else if (!rejectObservation) {
            // TODO...
            Debug.LogError("Observation rejected: No candidate found for the landmark association !");
        }

        return -1;
    }

    private Vector<float> g(float x, float y, float phi, float rf, float thetaf) {
        float cosphi = Mathf.Cos(phi);
        float sinphi = Mathf.Sin(phi);

        return V.Dense(new float[] {
            x + lidarA * cosphi - lidarB * sinphi + rf * Mathf.Cos(phi + thetaf),
            y + lidarA * sinphi + lidarB * cosphi + rf * Mathf.Sin(phi + thetaf)});
    }

    // Compute the gradient of g, relatively to x, y and phi:
    private Matrix<float> computeGradGxyp(float phi, float rf, float thetaf) {
        float cosphi = Mathf.Cos(phi);
        float sinphi = Mathf.Sin(phi);

        float dGx_dphi = -lidarA * sinphi - lidarB * cosphi - rf * Mathf.Sin(phi + thetaf);
        float dGy_dphi = lidarA * cosphi - lidarB * sinphi + rf * Mathf.Cos(phi + thetaf);

        return M.DenseOfArray(new float[,] {
            { 1, 0, dGx_dphi},
            { 0, 1, dGy_dphi } });
    }

    // Compute the gradient of g, relatively to rf and thetaf:
    private Matrix<float> computeGradGrt(float phi, float rf, float thetaf) {
        float cosphi_thetaf = Mathf.Cos(phi + thetaf);
        float sinphi_thetaf = Mathf.Sin(phi + thetaf);

        return M.DenseOfArray(new float[,] {
            { cosphi_thetaf, -rf * sinphi_thetaf },
            { sinphi_thetaf, rf * cosphi_thetaf } });
    }

    // Given the previous state estimate and the current inputs, compute the prediction 
    // of the new state estimate according to equation (10)
    private ModelState predictStateEstimate(ModelState previous, ModelInputs inputs, float deltaT) {
        float x = previous.x + deltaT * inputs.V * Mathf.Cos(previous.phi);
        float y = previous.y + deltaT * inputs.V * Mathf.Sin(previous.phi);
        float phi = previous.phi + deltaT * inputs.V * Mathf.Tan(inputs.gamma) / vehicleL;

        return new ModelState(x, y, phi);
    }

    // Given the estimated position and the landmark associated to the observation, predict what the observation
    // should be, according to equations (11) and (37):
    private Observation predictObservation(ModelState predictedState, int landmarkIndex) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = lidarA, b = lidarB;

        float xr = predictedState.x + a * cosphi - b * sinphi;
        float yr = predictedState.y + a * sinphi + b * cosphi;

        float dX = confirmedLandmarks[landmarkIndex].x - xr;
        float dY = confirmedLandmarks[landmarkIndex].y - yr;

        float ri = Mathf.Sqrt(dX * dX + dY * dY);
        float thetai = Mathf.Atan2(dY, dX) - predictedState.phi;

        return new Observation(ri, thetai);
    }

    // Given the previous state estimate and the current inputs, compute Fv, the Jacobian of f():
    private Matrix<float> computeFv(ModelState previous, ModelInputs inputs, float deltaT) {
        float cosphi = Mathf.Cos(previous.phi);
        float sinphi = Mathf.Sin(previous.phi);

        return M.DenseOfArray(new float[,] {
            {1, 0, -deltaT*inputs.V*sinphi},            // Derivative f1(x, y, phi) / dx
            {0, 1, deltaT*inputs.V*cosphi},             // Derivative f2(x, y, phi) / dy
            {0, 0, 1 } });                              // Derivative f3(x, y, phi) / dphi
    }

    private Matrix<float> computeHi(ModelState predictedState, int landmarkIndex, int landmarkCount) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = lidarA, b = lidarB;

        float xi = confirmedLandmarks[landmarkIndex].x;
        float yi = confirmedLandmarks[landmarkIndex].y;
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

        Matrix<float> Hi = M.Sparse(OBSERVATION_DIM, STATE_DIM + landmarkCount * LANDMARK_DIM);
        Hi.SetSubMatrix(0, 0, Hv);
        Hi.SetSubMatrix(0, STATE_DIM + landmarkIndex * LANDMARK_DIM, Hpi);

        return Hi;
    }

    public ModelState getStateEstimate() {
        return stateEstimate.copy();
    }
}
