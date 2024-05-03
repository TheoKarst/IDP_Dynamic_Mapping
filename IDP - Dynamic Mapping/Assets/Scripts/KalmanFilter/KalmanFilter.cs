using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using static RobotController;

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
    private List<PotentialLandmark> potentialLandmarks = new List<PotentialLandmark>();

    // Covariance matrices for the process noise and the observations:
    private Matrix<float> Q, R;

    // Current timestep k (number of times we called the updateStateEstimate() function). This is used to
    // find which potential landmarks are stable enough to be considered as confirmed landmarks:
    private long timestep = 0;

    // Current state estimate, and state covariance estimate of the robot:
    private ModelState stateEstimate;                        // x_hat(k|k)
    private StateCovariance stateCovarianceEstimate;         // P(k|k)

    // This represent the state estimate with state prediction only
    // (without using Kalman filter update from observations). This is used to show the impact of the EKF
    // on the state estimate:
    private ModelState stateEstimateNoUpdate;

    // Save the last time of an update to determine the elapsed time between two updates:
    private float lastTimeUpdate = -1;

    // Local position of the lidar and length of the vehicle, as defined in Dissanayake's paper:
    private ModelParams model;

    // Log file to write the data from the Kalman Filter:
    private StreamWriter logFile;

    // Use a pointer to the robot controller to have the real state of the robot (only used for logs):
    private RobotController controller;

    // TESTING: Save the observations to draw them:
    private List<Vector<float>> observationsPos;
    private List<Matrix<float>> observationsCov;

    public KalmanFilter(RobotController controller, ModelState initialState, 
        ModelParams model, StreamWriter logFile) {

        this.controller = controller;
        this.model = model;
        this.logFile = logFile;

        if(logFile != null)
            logFile.WriteLine("time;real_x;predict_x;update_x;real_y;predict_y;update_y;real_phi;predict_phi;update_phi");

        float eX = model.errorX;
        float eY = model.errorY;
        float ePhi = model.errorPhi;
        float eR = model.errorR;
        float eTheta = model.errorTheta;
        Debug.Log("Error estimates: (eX, eY, ePhi°, eR, eTheta°) = (" 
            + eX + ", " + eY + ", " + Mathf.Rad2Deg * ePhi + ", " + eR + ", " + Mathf.Rad2Deg * eTheta + ")");

        Q = M.Diagonal(new float[] {eX*eX, eY*eY, ePhi*ePhi});      // Covariance matrix for the process noise errors (x, y, phi)
        R = M.Diagonal(new float[] {eR*eR, eTheta*eTheta});         // Covariance matrix for the observation errors (r, theta)

        // Initialize the state estimate with the given start position of the robot:
        stateEstimate = initialState;
        stateEstimateNoUpdate = initialState;
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

    // For debugging, draw cubes to represent the position of the robot (red), and
    // the confirmed (green) and potential (yellow) landmarks:
    public void drawGizmos(bool drawConfirmedLandmarks, bool drawPotentialLandmarks, bool drawObservations) {
        Matrix<float> cov;
        Vector<float> position;

        // Position and error estimate of the robot:
        Gizmos.color = Color.red;
        cov = stateCovarianceEstimate.extractPvv();
        Vector3 robotCenter = new Vector3(stateEstimate.x, 0.5f, stateEstimate.y);
        Gizmos.DrawCube(robotCenter,
                        new Vector3(Mathf.Sqrt(cov[0,0]), Mathf.Sqrt(cov[2,2]), Mathf.Sqrt(cov[1,1])));

        Vector3 direction = Quaternion.AngleAxis(-Mathf.Rad2Deg * stateEstimate.phi, Vector3.up)
            * Vector3.right;
        Gizmos.DrawLine(robotCenter, robotCenter + direction);

        if (drawConfirmedLandmarks) {
            // Position and error estimate of the confirmed landmarks:
            for (int i = 0; i < confirmedLandmarks.Count; i++) {
                position = confirmedLandmarks[i].getPosition();
                cov = stateCovarianceEstimate.extractLandmarkCovariance(i);

                Gizmos.color = Color.green;
                Gizmos.DrawCube(new Vector3(position[0], 0.5f, position[1]),
                                new Vector3(Mathf.Sqrt(cov[0, 0]), 0, Mathf.Sqrt(cov[1, 1])));
            }
        }

        if(drawPotentialLandmarks) {
            // Position and error estimate of the potential landmarks:
            Gizmos.color = Color.red;
            foreach (PotentialLandmark landmark in potentialLandmarks) {
                position = landmark.getPosition();
                cov = landmark.getCovariance();

                Gizmos.DrawCube(new Vector3(position[0], 0.5f, position[1]),
                                new Vector3(Mathf.Sqrt(cov[0, 0]), 0, Mathf.Sqrt(cov[1, 1])));
            }
        }

        if (observationsPos != null && drawObservations) {
            // Also draw the position of the observations position estimates:
            for (int i = 0; i < observationsPos.Count; i++) {
                position = observationsPos[i];
                cov = observationsCov[i];

                Gizmos.color = Color.blue;
                Gizmos.DrawCube(new Vector3(position[0], 0.5f, position[1]),
                                new Vector3(0.1f, 0, 0.1f)); // new Vector3(Mathf.Sqrt(cov[0, 0]), 0, Mathf.Sqrt(cov[1, 1])));
            }
        }
    }

    // Given observations from the sensor of the robot, the inputs of the robot and the current time,
    // try to associate the observations with known landmarks and update the robot state using the
    // Extended Kalman Filter. The observation may be empty (no interesting candidate). In this case,
    // we just perform a state prediction without update:
    public void updateStateEstimate(Observation[] observations, ModelInputs inputs, float currentTime) {
        float deltaT = lastTimeUpdate < 0 ? 0 : currentTime - lastTimeUpdate;
        lastTimeUpdate = currentTime;
        timestep++;

        //// 1. Prediction: Predict the current state estimate from our previous estimate and the current inputs

        // Equation (10): From the previous state estimate and current inputs,
        // predict a new state estimate: x_hat(k+1|k)
        ModelState statePrediction = predictStateEstimate(stateEstimate, inputs, deltaT);

        Debug.Log("Estimate: " + stateEstimate + "; Prediction: " + statePrediction
            + "; Confirmed landmarks: " + confirmedLandmarks.Count + "; Potential: " + potentialLandmarks.Count);

        // This is used to compare EKF update with prediction only:
        stateEstimateNoUpdate = predictStateEstimate(stateEstimateNoUpdate, inputs, deltaT);
        
        // Equation (12): From the previous state estimate and the current inputs, compute the new estimate of the
        // state covariance matrix: P(k+1|k)
        Matrix<float> Fv = computeFv(stateEstimate, inputs, deltaT);
        StateCovariance stateCovariancePrediction = stateCovarianceEstimate.predictStateEstimateCovariance(Fv, Q);

        observationsPos = new List<Vector<float>>();
        observationsCov = new List<Matrix<float>>();

        //// 2. Observation: Match each observation with a landmark, and use the error to update the state estimate: 
        List<(Observation, int)> landmarkAssociation = new List<(Observation, int)>();

        foreach(Observation observation in observations) {
            
            // Use the landmark association algorithm defined in the Appendix II to associate our
            // observation with a possible landmark:
            int landmarkIndex = computeLandmarkAssociation(observation, statePrediction, 
                                                            stateCovariancePrediction);

            // If we have a valid landmark index, add this observation and the associated landmark
            // index to the list:
            if (landmarkIndex != -1)
                landmarkAssociation.Add((observation, landmarkIndex));
        }

        // Update the set of landmarks:
        updatePotentialLandmarks(stateCovariancePrediction);
        
        // If no candidate landmark was found for our observation, ignore the update procedure and use
        // the state prediction and covariance prediction as the new state and covariance estimates:
        if (landmarkAssociation.Count == 0) {
            stateEstimate = statePrediction;
            stateCovarianceEstimate = stateCovariancePrediction;

            if (logFile != null) {
                ModelState realState = controller.getRobotRealState();
                float realPhi = principalDegreeMeasure(realState.phi);
                float predictPhi = principalDegreeMeasure(stateEstimateNoUpdate.phi);
                float estimatePhi = principalDegreeMeasure(stateEstimate.phi);

                logFile.WriteLine(currentTime + ";"
                                + realState.x + ";" + stateEstimateNoUpdate.x + ";" + stateEstimate.x + ";"
                                + realState.y + ";" + stateEstimateNoUpdate.y + ";" + stateEstimate.y + ";"
                                + realPhi + ";" + predictPhi + ";" + estimatePhi);
            }

            return;
        }

        // Stack all the observations that were matched with a landmark to update the state:
        int stackSize = landmarkAssociation.Count * OBSERVATION_DIM;

        // Create an innovation stack (stack of observation - observationPrecdiction):
        Vector<float> innovationStack = V.Dense(stackSize);

        // Also create a stack of observation jacobians Hstack, and a stack of observation noise Rstack:
        Matrix<float> Hstack = M.Sparse(stackSize, STATE_DIM + confirmedLandmarks.Count * LANDMARK_DIM);
        Matrix<float> Rstack = M.Sparse(stackSize, stackSize);

        for(int i = 0; i < landmarkAssociation.Count; i++) {
            Observation observation; int landmarkIndex;
            (observation, landmarkIndex) = landmarkAssociation[i];

            // Equation (11): From the state prediction, and for each observation that was matched with
            // a landmark, predict what the observation should be: z_hat(k+1|k)
            Observation observationPrediction = predictObservation(statePrediction, landmarkIndex);

            // Compare the expected observation with the real one to compute the innovation vector:
            Observation.substract(observation, observationPrediction, innovationStack, OBSERVATION_DIM * i);

            // Debug: Draw the expected observation and the real observation:
            controller.lidar.DrawObservation(observationPrediction, Color.blue);
            controller.lidar.DrawObservation(observation, Color.green);

            // Compute Hi, the observation Jacobian associated with this landmark using equation (37),
            // and stack it into Hstack:
            computeHi(statePrediction, landmarkIndex, Hstack, OBSERVATION_DIM * i);

            // Also update the stack observation noise:
            Rstack.SetSubMatrix(i * OBSERVATION_DIM, i * OBSERVATION_DIM, R);
        }

        // Finally compute the innovation covariance matrix using Hstack and Rstack (Equation 14):
        Matrix<float> S, W;
        (S, W) = stateCovariancePrediction.computeInnovationAndGainMatrices(Hstack, Rstack);

        //// 3. Update: Update the state estimate from our observation
        stateEstimate = statePrediction + W * innovationStack;                                  // Equation (15)
        stateCovarianceEstimate = stateCovariancePrediction - W * S.TransposeAndMultiply(W);    // Equation (16)

        // Write data to the log file:
        if(logFile != null) {
            ModelState realState = controller.getRobotRealState();
            float realPhi = principalDegreeMeasure(realState.phi);
            float predictPhi = principalDegreeMeasure(stateEstimateNoUpdate.phi);
            float updatePhi = principalDegreeMeasure(stateEstimate.phi);

            logFile.WriteLine(currentTime + ";"
                            + realState.x + ";" + stateEstimateNoUpdate.x + ";" + stateEstimate.x + ";"
                            + realState.y + ";" + stateEstimateNoUpdate.y + ";" + stateEstimate.y + ";"
                            + realPhi + ";" + predictPhi + ";" + updatePhi);
        }
    }

    private float principalDegreeMeasure(float angle) {
        while (angle > Mathf.PI)  angle -= 2 * Mathf.PI;
        while (angle < -Mathf.PI) angle += 2 * Mathf.PI;

        return angle * Mathf.Rad2Deg;
    }

    // From an observation of the LIDAR, the predicted state, the predicted state covariance estimate
    // and the previous landmarks, return the index of the landmark that is the most likely to correspond
    // to the observation. If no appropriate landmark is found, the set of potential landmarks is updated.
    // return -1 if there is no landmark candidate for the observation
    private int computeLandmarkAssociation(Observation observation, ModelState statePrediction, StateCovariance stateCovariancePrediction) {
        // Perform some renamings for readability:
        float xk = statePrediction.x, yk = statePrediction.y, phik = statePrediction.phi;
        float rf = observation.r, thetaf = observation.theta;

        // Covariance matrix of the vehicle location estimate, extracted from P(k|k):
        Matrix<float> Pv = stateCovariancePrediction.extractPvv();       // Matrix(STATE_DIM, STATE_DIM)

        // 1. Compute pf, the position of the landmark possibly responsible for this observation:
        Vector<float> pf = g(xk, yk, phik, rf, thetaf);                // Matrix(LANDMARK_DIM, 1)
        
        // Compute Pf, the covariance matrix of pf:
        Matrix<float> gradGxyp = computeGradGxyp(phik, rf, thetaf);    // Matrix(LANDMARK_DIM, STATE_DIM)
        Matrix<float> gradGrt = computeGradGrt(phik, rf, thetaf);      // Matrix(LANDMARK_DIM, OBSERVATION_DIM)

        // Pv = Matrix(STATE_DIM, STATE_DIM) and R = M(OBSERVATION_DIM, OBSERVATION_DIM)
        // So Pf = Matrix(LANDMARK_DIM, LANDMARK_DIM):
        Matrix<float> Pf = gradGxyp * Pv.TransposeAndMultiply(gradGxyp)
                        + gradGrt * R.TransposeAndMultiply(gradGrt);

        // Used for drawing purposes:
        observationsPos.Add(pf);
        observationsCov.Add(Pf);

        // Compute the minimum euclidean distance between the observation and all the confirmed and potential landmarks:
        float minEuclideanDistance = -1;

        // 2. Try to associate the observation with a confirmed landmark:
        int acceptedLandmark = -1;
        for (int i = 0; i < confirmedLandmarks.Count; i++) {
            Vector<float> pi = confirmedLandmarks[i].getPosition();
            Matrix<float> Pi = stateCovariancePrediction.extractLandmarkCovariance(i);

            Vector<float> X = pf - pi;
            float dfi = (X.ToRowMatrix() * (Pf + Pi).Inverse() * X)[0];
            float euclideanDistance = (float) X.L2Norm();

            // If minEuclideanDistance wasn't initialised or if the current distance is a new minimum,
            // update the minimum:
            if (minEuclideanDistance < 0 || euclideanDistance < minEuclideanDistance)
                minEuclideanDistance = euclideanDistance;

            // If dfi < dmin, then the current landmark is chosen as a candidate for the observation:
            if (dfi < 1f) {
                // If no other landmark was accepted before, then accept this landmark:
                if (acceptedLandmark == -1) acceptedLandmark = i;

                // Else if more than one landmark is accepted for this observation, reject the observation:
                else {
                    Debug.LogError("Observation rejected: more than one landmark corresponds to this observation !");
                    return -1;
                }
            }
        }

        // If a confirmed landmark was accepted for the observation, use it to generate a new state estimate:
        if (acceptedLandmark != -1)
            return acceptedLandmark;

        // 3. Else we have to check the observation against the set of potential landmarks:
        for (int i = 0; i < potentialLandmarks.Count; i++) {
            Vector<float> pi = potentialLandmarks[i].getPosition();
            Matrix<float> Pi = potentialLandmarks[i].getCovariance();

            Vector<float> X = pf - pi;
            float dfi = (X.ToRowMatrix() * (Pf + Pi).Inverse() * X)[0];
            float euclideanDistance = (float) X.L2Norm();

            // If minEuclideanDistance wasn't initialised or if the current distance is a new minimum,
            // update the minimum:
            if (minEuclideanDistance < 0 || euclideanDistance < minEuclideanDistance)
                minEuclideanDistance = euclideanDistance;

            // If dfi < dmin, then the current landmark is chosen as a candidate for the observation:
            if (dfi < 0.1f) {
                // If no other landmark was accepted before, then accept this landmark:
                if (acceptedLandmark == -1) acceptedLandmark = i;

                // Else if more than one landmark is accepted for this observation, reject the observation:
                else {
                    Debug.LogError("Observation rejected: more than one potential landmark corresponds to this observation !");
                    return -1;
                }
            }
        }

        // If a potential landmark was accepted for the observation, use the observation to update its state:
        if (acceptedLandmark != -1) {
            potentialLandmarks[acceptedLandmark].updateState(pf, Pf);
        }

        // 4. Else if the observation is far enough from the nearest landmark, we can add
        // it to the set of potential landmarks:
        else if (minEuclideanDistance < 0 || minEuclideanDistance > model.minDistanceBetweenLandmarks) {
            potentialLandmarks.Add(new PotentialLandmark(pf, Pf, timestep));
        }

        // Even if we found a potential landmark that matches the observation, still don't use it to update
        // the state estimate, since this may be an unstable landmark:
        return -1;
    }

    // Step 5. of the Appendix II: Examine the set of potential landmarks, remove the unused ones and
    // confirm the most stable ones:
    private void updatePotentialLandmarks(StateCovariance stateCovariancePrediction) {
        const int cmin = 5;         // Number of associations to consider a potential landmark as stable
        const int tmax = 50;        // Number of timesteps after which an unstable potential landmark is removed

        List<PotentialLandmark> newPotentialLandmarks = new List<PotentialLandmark>();

        for (int i = 0; i < potentialLandmarks.Count; i++) {
            PotentialLandmark current = potentialLandmarks[i];

            // a. If the potential landmark is stable enough, add it to the list of confirmed landmarks:
            if (current.getAssociationsCount() > cmin) {
                confirmedLandmarks.Add(current.toLandmark());
                stateCovariancePrediction.addLandmark(current.getCovariance());
            }

            // b. Keep this potential landmark in the list only if the landmark is recent enough:
            else if (timestep - current.getCreationTime() < tmax) {
                newPotentialLandmarks.Add(current);
            }
        }

        potentialLandmarks = newPotentialLandmarks;
    }

    private Vector<float> g(float x, float y, float phi, float rf, float thetaf) {
        float a = model.a, b = model.b;
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);

        return V.Dense(new float[] {
            x + a * cosphi - b * sinphi + rf * Mathf.Cos(phi + thetaf),
            y + a * sinphi + b * cosphi + rf * Mathf.Sin(phi + thetaf)});
    }

    // Compute the gradient of g, relatively to x, y and phi:
    private Matrix<float> computeGradGxyp(float phi, float rf, float thetaf) {
        float a = model.a, b = model.b;
        float cosphi = Mathf.Cos(phi), sinphi = Mathf.Sin(phi);

        float dGx_dphi = -a * sinphi - b * cosphi - rf * Mathf.Sin(phi + thetaf);
        float dGy_dphi = a * cosphi - b * sinphi + rf * Mathf.Cos(phi + thetaf);

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
        float phi = previous.phi + deltaT * inputs.V * Mathf.Tan(inputs.gamma) / model.L;

        return new ModelState(x, y, phi);
    }

    // Given the estimated position and the landmark associated to the observation, predict what the observation
    // should be, according to equations (11) and (37):
    private Observation predictObservation(ModelState predictedState, int landmarkIndex) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = model.a, b = model.b;

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

    private void computeHi(ModelState predictedState, int landmarkIndex, Matrix<float> dest, int index) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = model.a, b = model.b;

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

        dest.SetSubMatrix(index, 0, Hv);
        dest.SetSubMatrix(index, STATE_DIM + landmarkIndex * LANDMARK_DIM, Hpi);
    }

    public ModelState getStateEstimate() {
        return stateEstimate.copy();
    }
}
