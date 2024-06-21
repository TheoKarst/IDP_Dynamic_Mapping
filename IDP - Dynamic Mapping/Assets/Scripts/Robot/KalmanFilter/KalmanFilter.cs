using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

public class KalmanFilter {
    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<double> M = Matrix<double>.Build;
    private static VectorBuilder<double> V = Vector<double>.Build;

    // Create shortcuts for state, landmarks and observations dimensions:
    private const int STATE_DIM = VehicleState.DIMENSION;        // (x, y, phi)
    private const int LANDMARK_DIM = Landmark.DIMENSION;         // (x, y)
    private const int OBSERVATION_DIM = Observation.DIMENSION;   // (r, theta)

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Parameters of the Kalman Filter:

    // Number of associations to consider a potential landmark as stable:
    const int AssociationsForStableLandmark = 30;        // Was 6

    // Number of timesteps after which an unstable potential landmark is removed
    const int PotentialLandmarkLifetime = 60;           // Was 51

    // Maximal Mahalanobis distance between an observation and a confirmed landmark to match them together:
    const float MaxNormDistanceConfirmedLandmarks = 0.1f;

    // Maximal Mahalanobis distance between an observation and a potential landmark to match them together:
    const float MaxNormDistancePotentialLandmarks = 0.1f;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Index of the LIDAR we want to use for localisation (for now, using multiple LIDARs at the same
    // time in the Kalman Filter is not supported):
    private int lidarIndex;

    // Model of the vehicle (used to perform all the operations specific to the vehicle model):
    private VehicleModel vehicleModel;
    
    // According to the appendix II, create two lists of landmarks:
    private List<Landmark> confirmedLandmarks = new List<Landmark>();
    private List<PotentialLandmark> potentialLandmarks = new List<PotentialLandmark>();

    // The maximum number of landmarks we can use to update the robot state estimate
    // (more landmarks means more precision, but is also slower):
    private int maxLandmarksPerUpdate;

    // Minimum distance between landmarks (prevent having too many landmarks at the same position):
    private float minDistanceBetweenLandmarks;

    // Save the last time of an update to determine the elapsed time between two updates:
    private float lastTimeUpdate = -1;

    // Current timestep k (number of times we called the updateStateEstimate() function). This is used to
    // find which potential landmarks are stable enough to be considered as confirmed landmarks:
    private long timestep = 0;

    // Current state estimate, and state covariance estimate of the robot:
    private VehicleState stateEstimate;                        // x_hat(k|k)
    private StateCovariance stateCovarianceEstimate;           // P(k|k)

    // This represent the state estimate with state prediction only
    // (without using Kalman filter update from observations). This is used to show the impact of the EKF
    // on the state estimate:
    private VehicleState statePredictionOnly;

    // Use a reference to the simulated robot to have the real state of the robot (only used for logs):
    private KalmanRobot robot;

    // Debugging: Save the observations to draw them:
    private List<Vector<double>> observationsPos;
    private List<Matrix<double>> observationsCov;

    // Used to print debug messages in the console / in a text file:
    private Logger logger;

    public KalmanFilter(KalmanRobot robot, VehicleState initialState, VehicleModel vehicleModel, 
        KalmanParams parameters, Logger logger, int lidarIndex) {

        this.robot = robot;
        this.lidarIndex = lidarIndex;

        // Initialize the state estimate with the given start position of the robot:
        this.stateEstimate = initialState;
        this.statePredictionOnly = initialState;
        this.stateCovarianceEstimate = new StateCovariance(STATE_DIM, 0, LANDMARK_DIM);

        this.vehicleModel = vehicleModel;

        this.maxLandmarksPerUpdate = parameters.maxLandmarksPerUpdate;
        this.minDistanceBetweenLandmarks = parameters.minDistanceBetweenLandmarks;

        this.logger = logger;
        logger.Log("time;real_x;real_y;real_phi;predict_x;predict_y;predict_phi;update_x;update_y;update_phi");
    }

    // For debugging, draw cubes to represent the position of the robot (red), and
    // the confirmed (green) and potential (yellow) landmarks:
    public void DrawGizmos(bool drawConfirmedLandmarks, bool drawPotentialLandmarks, bool drawObservations) {
        Matrix<double> cov;
        Vector<double> position;

        // Position and error estimate of the robot:
        // Gizmos.color = Color.yellow;
        // Vector3 robotCenter = new Vector3(statePredictionOnly.x, 0.2f, statePredictionOnly.y);
        // Gizmos.DrawSphere(robotCenter, 0.1f);

        Gizmos.color = Color.red;
        cov = stateCovarianceEstimate.ExtractPvv();
        Vector3 robotCenter = new Vector3(stateEstimate.x, 0.11f, stateEstimate.y);
        Gizmos.DrawCube(robotCenter, new Vector3(
            Mathf.Sqrt((float) cov[0,0]), 
            Mathf.Sqrt((float) cov[2,2]), 
            Mathf.Sqrt((float) cov[1,1])));

        Vector3 direction = Quaternion.AngleAxis(-Mathf.Rad2Deg * stateEstimate.phi, Vector3.up)
            * Vector3.right;
        Gizmos.DrawLine(robotCenter, robotCenter + direction);

        if (drawConfirmedLandmarks) {
            // Position and error estimate of the confirmed landmarks:
            for (int i = 0; i < confirmedLandmarks.Count; i++) {
                position = confirmedLandmarks[i].getPosition();
                cov = stateCovarianceEstimate.ExtractLandmarkCovariance(i);

                Gizmos.color = Color.green;
                // Gizmos.DrawCube(new Vector3((float) position[0], 0.5f, (float) position[1]),
                //                new Vector3(Mathf.Sqrt((float) cov[0, 0]), 0, Mathf.Sqrt((float) cov[1, 1])));

                Gizmos.DrawSphere(new Vector3((float)position[0], 0.5f, (float)position[1]), 0.1f);
            }
        }

        if(drawPotentialLandmarks) {
            // Position and error estimate of the potential landmarks:
            Gizmos.color = Color.red;
            foreach (PotentialLandmark landmark in potentialLandmarks) {
                position = landmark.getPosition();
                cov = landmark.getCovariance();

                Gizmos.DrawCube(new Vector3((float) position[0], 0.5f, (float) position[1]),
                                new Vector3(Mathf.Sqrt((float) cov[0, 0]), 0, Mathf.Sqrt((float) cov[1, 1])));
            }
        }

        if (observationsPos != null && drawObservations) {
            // Also draw the position of the observations position estimates:
            for (int i = 0; i < observationsPos.Count; i++) {
                position = observationsPos[i];
                cov = observationsCov[i];

                Gizmos.color = Color.blue;
                Gizmos.DrawCube(new Vector3((float) position[0], 0.5f, (float) position[1]),
                                new Vector3(0.1f, 0, 0.1f)); // new Vector3(Mathf.Sqrt(cov[0, 0]), 0, Mathf.Sqrt(cov[1, 1])));
            }
        }
    }

    // Given observations from the sensor of the robot, the inputs of the robot and the current time,
    // try to associate the observations with known landmarks and update the robot state using the
    // Extended Kalman Filter. The observation may be empty (no interesting candidate). In this case,
    // we just perform a state prediction without update:
    public void UpdateStateEstimate(List<Observation> observations, ModelInputs inputs, float currentTime) {
        float deltaT = lastTimeUpdate < 0 ? 0 : currentTime - lastTimeUpdate;
        lastTimeUpdate = currentTime;
        timestep++;

        //// 1. Prediction: Predict the current state estimate from our previous estimate and the current inputs

        // Equation (10): From the previous state estimate and current inputs,
        // predict a new state estimate: x_hat(k+1|k)
        VehicleState statePrediction = vehicleModel.PredictCurrentState(stateEstimate, inputs, deltaT);

        // This is used to compare EKF update with prediction only:
        statePredictionOnly = vehicleModel.PredictCurrentState(statePredictionOnly, inputs, deltaT);
        
        // Equation (12): From the previous state estimate and the current inputs, compute the new estimate of the
        // state covariance matrix: P(k+1|k)
        Matrix<double> Fv = vehicleModel.StatePredictionJacobian(stateEstimate, inputs, deltaT);
        StateCovariance stateCovariancePrediction 
            = stateCovarianceEstimate.PredictStateEstimateCovariance(Fv, vehicleModel);

        observationsPos = new List<Vector<double>>();
        observationsCov = new List<Matrix<double>>();

        //// 2. Observation: Match each observation with a landmark, and use the errors to update the state estimate: 
        List<(Observation, int)> landmarkAssociation = new List<(Observation, int)>();

        Profiler.BeginSample("Landmarks Association");
        foreach (Observation observation in observations) {
            
            // Use the landmark association algorithm defined in the Appendix II to associate our
            // observation with a possible landmark:
            int landmarkIndex = ComputeLandmarkAssociation(observation, statePrediction, 
                                                            stateCovariancePrediction);

            // If we have a valid landmark index, add this observation and the associated landmark
            // index to the list:
            if (landmarkIndex != -1) {
                landmarkAssociation.Add((observation, landmarkIndex));

                // If we have enough landmarks, we can stop here:
                if (landmarkAssociation.Count >= maxLandmarksPerUpdate)
                    break;
            }
        }
        Profiler.EndSample();

        // Update the set of landmarks:
        UpdatePotentialLandmarks(stateCovariancePrediction);
        
        // If no candidate landmark was found for our observation, ignore the update procedure and use
        // the state prediction and covariance prediction as the new state and covariance estimates:
        if (landmarkAssociation.Count == 0) {
            stateEstimate = statePrediction;
            stateCovarianceEstimate = stateCovariancePrediction;

            logger.Log(robot.GetRobotRealState(), statePredictionOnly, stateEstimate);
            return;
        }

        // Stack all the observations that were matched with a landmark to update the state:
        int stackSize = landmarkAssociation.Count * OBSERVATION_DIM;

        // Create an innovation stack (stack of observation - observationPrecdiction):
        Vector<double> innovationStack = V.Dense(stackSize);

        // Also create a stack of observation jacobians Hstack, and a stack of observation noise Rstack:
        Matrix<double> Hstack = M.Sparse(stackSize, STATE_DIM + confirmedLandmarks.Count * LANDMARK_DIM);
        Matrix<double> Rstack = M.Sparse(stackSize, stackSize);

        Profiler.BeginSample("Build Hstack, Rstack");
        for(int i = 0; i < landmarkAssociation.Count; i++) {
            Observation observation; int landmarkIndex;
            (observation, landmarkIndex) = landmarkAssociation[i];
            Landmark landmark = confirmedLandmarks[landmarkIndex];

            // Equation (11): From the state prediction, and for each observation that was matched with
            // a landmark, predict what the observation should be: z_hat(k+1|k)
            Observation observationPrediction 
                = vehicleModel.PredictObservation(statePrediction, landmark.x, landmark.y, lidarIndex);

            // Compare the expected observation with the real one to compute the innovation vector:
            Observation.Substract(observation, observationPrediction, innovationStack, OBSERVATION_DIM * i);

            // Debug: Draw the expected observation and the real observation:
            robot.GetLidar().DrawObservation(observationPrediction, Color.blue);
            robot.GetLidar().DrawObservation(observation, Color.green);

            // Compute Hi, the observation Jacobian associated with this landmark using equation (37),
            // and stack it into Hstack:
            vehicleModel.ComputeHi(statePrediction, landmark, landmarkIndex, Hstack, OBSERVATION_DIM * i, lidarIndex);

            // Also update the stack observation noise:
            Rstack.SetSubMatrix(i * OBSERVATION_DIM, i * OBSERVATION_DIM, vehicleModel.ObservationError);
        }
        Profiler.EndSample();

        Profiler.BeginSample("Compute innovation");
        // Finally compute the innovation covariance matrix using Hstack and Rstack (Equation 14):
        Matrix<double> S, W;
        (S, W) = stateCovariancePrediction.ComputeInnovationAndGainMatrices(Hstack, Rstack);
        Profiler.EndSample();

        //// 3. Update: Update the state estimate from our observation
        Profiler.BeginSample("Update");
        stateEstimate = statePrediction + W * innovationStack;                                  // Equation (15)
        stateCovarianceEstimate = stateCovariancePrediction - W * S.TransposeAndMultiply(W);    // Equation (16)
        Profiler.EndSample();

        // Write data to the log file:
        logger.Log(robot.GetRobotRealState(), statePredictionOnly, stateEstimate);
    }

    // From an observation of the LIDAR, the predicted state, the predicted state covariance estimate
    // and the previous landmarks, return the index of the landmark that is the most likely to correspond
    // to the observation. If no appropriate landmark is found, the set of potential landmarks is updated.
    // return -1 if there is no landmark candidate for the observation
    private int ComputeLandmarkAssociation(Observation observation, VehicleState statePrediction, StateCovariance stateCovariancePrediction) {

        // Covariance matrix of the vehicle location estimate, extracted from P(k|k):
        Matrix<double> Pv = stateCovariancePrediction.ExtractPvv();

        // 1. Compute pf (the position of the landmark possibly responsible for the given observation),
        // and Pf (the corresponding covariance matrix):
        Vector<double> pf; Matrix<double> Pf;
        (pf, Pf) = vehicleModel.ComputeObservationPositionEstimate(statePrediction, Pv, observation);

        // Used for drawing purposes:
        observationsPos.Add(pf); observationsCov.Add(Pf);

        // Compute the minimum euclidean distance between the observation and all the confirmed and potential landmarks:
        float minEuclideanDistance = -1;

        // 2. Try to associate the observation with a confirmed landmark:
        int acceptedLandmark = -1;
        for (int i = 0; i < confirmedLandmarks.Count; i++) {
            Vector<double> pi = confirmedLandmarks[i].getPosition();
            Matrix<double> Pi = stateCovariancePrediction.ExtractLandmarkCovariance(i);

            // Compute the Mahalanobis distance between the observation and the landmark n°i:
            Vector<double> X = pf - pi;
            float dfi = (float) (X.ToRowMatrix() * (Pf + Pi).Inverse() * X)[0];
            float euclideanDistance = (float) X.L2Norm();

            // If minEuclideanDistance wasn't initialised or if the current distance is a new minimum,
            // update the minimum:
            if (minEuclideanDistance < 0 || euclideanDistance < minEuclideanDistance)
                minEuclideanDistance = euclideanDistance;

            // If dfi < MaxNormDistanceConfirmedLandmarks, then the current landmark is chosen as
            // a candidate for the observation:
            if (dfi < MaxNormDistanceConfirmedLandmarks) {
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
            Vector<double> pi = potentialLandmarks[i].getPosition();
            Matrix<double> Pi = potentialLandmarks[i].getCovariance();

            Vector<double> X = pf - pi;
            float dfi = (float) (X.ToRowMatrix() * (Pf + Pi).Inverse() * X)[0];
            float euclideanDistance = (float) X.L2Norm();

            // If minEuclideanDistance wasn't initialised or if the current distance is a new minimum,
            // update the minimum:
            if (minEuclideanDistance < 0 || euclideanDistance < minEuclideanDistance)
                minEuclideanDistance = euclideanDistance;

            // If dfi < MaxNormDistancePotentialLandmarks, then the current landmark is chosen as a
            // candidate for the observation:
            if (dfi < MaxNormDistancePotentialLandmarks) {
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
        else if (minEuclideanDistance < 0 || minEuclideanDistance > minDistanceBetweenLandmarks) {
            potentialLandmarks.Add(new PotentialLandmark(pf, Pf, timestep));
        }

        // Even if we found a potential landmark that matches the observation, still don't use it to update
        // the state estimate, since this may be an unstable landmark:
        return -1;
    }

    // Step 5. of the Appendix II: Examine the set of potential landmarks, remove the unused ones and
    // confirm the most stable ones:
    private void UpdatePotentialLandmarks(StateCovariance stateCovariancePrediction) {
        
        List<PotentialLandmark> newPotentialLandmarks = new List<PotentialLandmark>();

        for (int i = 0; i < potentialLandmarks.Count; i++) {
            PotentialLandmark current = potentialLandmarks[i];

            // a. If the potential landmark is stable enough, add it to the list of confirmed landmarks:
            if (current.getAssociationsCount() >= AssociationsForStableLandmark) {
                confirmedLandmarks.Add(current.toLandmark());
                stateCovariancePrediction.AddLandmark(current.getCovariance());
            }

            // b. Keep this potential landmark in the list only if the landmark is recent enough:
            else if (timestep - current.getCreationTime() <= PotentialLandmarkLifetime) {
                newPotentialLandmarks.Add(current);
            }
        }

        potentialLandmarks = newPotentialLandmarks;
    }

    public VehicleState GetStateEstimate() {
        return stateEstimate;
    }

    public Matrix<double> GetStateCovarianceEstimate() {
        return stateCovarianceEstimate.ExtractPvv();
    }
}
