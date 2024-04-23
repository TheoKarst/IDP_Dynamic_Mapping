using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using System;

public class RobotController : MonoBehaviour
{
    // Define some constants for the dimensions of the state, landmarks and observations:
    private const int STATE_DIM = 3;            // (x, y, phi)
    private const int LANDMARK_DIM = 2;         // (x, y)
    private const int OBSERVATION_DIM = 2;      // (r, theta)


    // Matrix builder used as a shortcut for vector and matrix creation:
    private static MatrixBuilder<double> M = Matrix<double>.Build;
    private static VectorBuilder<double> V = Vector<double>.Build;

    public struct State {
        public float x;
        public float y;
        public float phi;

        public State(float x, float y, float phi) {
            this.x = x;
            this.y = y;
            this.phi = phi;
        }
    }

    public struct Landmark {
        public Vector<double> position;
        public Landmark(float x, float y) {
            position = V.Dense(new double[] { x, y });
        }

        public float x() { return (float) position[0]; }
        public float y() { return (float) position[1]; }
    }
    
    public struct Observation {
        public float r;
        public float theta;

        public Observation(float r, float theta) {
            this.r = r;
            this.theta = theta;
        }
    }

    public struct Inputs {
        public float V;
        public float gamma;

        public Inputs(float V, float gamma) {
            this.V = V;
            this.gamma = gamma;
        }
    }

    // Create a structure to represent P(i|j), which is a block matrix
    public class StateCovariance {
        /* P(i|j) = [ Pvv(i|j),              Pvm(i|j) ]
         *          [ Pvm(i|j).Transpose(),  Pmm(i|j) ]
         * 
         * Pvv(i|j): error covariance matrix associated with the vehicle state estimate
         * Pmm(i|j): map covariance matrix associated with the landmarks state estimates
         * Pvm(i|j): cross-covariance matrix between vehicle and landmark states
         */

        public Matrix<double> P;

        public Matrix<double> Pvv;      // M(STATE_DIM, STATE_DIM)
        public Matrix<double> Pvm;      // M(STATE_DIM, landmarksCount * LANDMARK_DIM)
        public Matrix<double> Pmm;      // M(landmarksCount * LANDMARK_DIM, landmarksCount * LANDMARK_DIM)

        public StateCovariance(int landmarksCount) {
            Pvv = M.DenseIdentity(STATE_DIM);
            Pvm = M.Dense(STATE_DIM, landmarksCount * LANDMARK_DIM, 0);
            Pmm = M.DenseIdentity(landmarksCount * LANDMARK_DIM);
        }

        public StateCovariance(Matrix<double> Pvv, Matrix<double> Pvm, Matrix<double> Pmm) {
            this.Pvv = Pvv;
            this.Pvm = Pvm;
            this.Pmm = Pmm;
        }

        public Matrix<double> extractLandmarkCovariance(int landmarkIndex) {
            return Pmm.SubMatrix(landmarkIndex * LANDMARK_DIM, LANDMARK_DIM, 
                                 landmarkIndex * LANDMARK_DIM, LANDMARK_DIM);
        }

        public StateCovariance predictStateEstimateCovariance(Matrix<double> Fv, Matrix<double> Q) {
            return new StateCovariance(
                Fv * Pvv.TransposeAndMultiply(Fv) + Q,
                Fv * Pvm,
                Pmm);
        }

        public (Matrix<double>, Matrix<double>) computeInnovationAndGainMatrices(int landmarkIndex, Matrix<double> Hv, Matrix<double> Hpi, Matrix<double> R) {

            // Hi = [Hv, 0...0, Hpi, 0...0] = [Hv, sparse]:
            Matrix<double> sparse = M.Sparse(OBSERVATION_DIM, Pmm.RowCount);
            sparse.SetSubMatrix(0, landmarkIndex * LANDMARK_DIM, Hpi);

            // Compute P * Hi.Transpose():
            Matrix<double> A = Pvv.TransposeAndMultiply(Hv) + Pvm.TransposeAndMultiply(sparse);
            Matrix<double> B = Pvm.TransposeThisAndMultiply(Hv.Transpose()) + Pmm.TransposeAndMultiply(sparse);

            // Innovation matrix:
            // Si = Hi * (P * Hi.Transpose()) + R = [Hv, sparse] * [A, B].Transpose() + R
            Matrix<double> Si = Hv * A + sparse * B + R;

            // Gain matrix:
            Matrix<double> Wi = M.Dense(STATE_DIM + Pmm.RowCount, OBSERVATION_DIM);
            Wi.SetSubMatrix(0, 0, A);           // Matrix(STATE_DIM, OBSERVATION_DIM)
            Wi.SetSubMatrix(STATE_DIM, 0, B);   // Matrix(landmarksCount * LANDMARK_DIM, OBSERVATION_DIM)
            Wi = Wi * Si.Inverse();

            return (Si, Wi);
        }
    }

    public Lidar lidar;

    public float acceleration = 10;
    public float L = 0.3f;

    public float maxSpeed = 0.015f;
    public float maxSteering = 85;

    public float friction = 0.02f;

    private float velocity = 0;
    private float steering = 0;

    // According to the appendix II, create two lists of landmarks:
    private List<Landmark> confirmedLandmarks = new List<Landmark>();
    private List<Landmark> potentialLandmarks = new List<Landmark>();

    // Covariance matrices for the process noise and the observations:
    private Matrix<double> Q, R;

    // Start is called before the first frame update
    void Start() {
        Q = M.DenseIdentity(STATE_DIM);             // Covariance matrix for the process noise errors (x, y, phi)
        R = M.DenseIdentity(OBSERVATION_DIM);       // Covariance matrix for the observation errors (r, theta)
    }

    // Update is called once per frame
    void Update() {
        float h = Time.deltaTime;

        // Update the velocity of the robot:
        if (Input.GetKey(KeyCode.UpArrow))
            velocity += h * acceleration;
        else if (Input.GetKey(KeyCode.DownArrow))
            velocity -= h * acceleration;
        else
            velocity -= h * velocity * friction;

        // Update the steering of the robot:
        if (Input.GetKey(KeyCode.LeftArrow))
            steering = -maxSteering;
        else if (Input.GetKey(KeyCode.RightArrow))
            steering = maxSteering;
        else
            steering = 0;

        // Clamp the velocity between boundaries:
        velocity = Mathf.Clamp(velocity, -maxSpeed, maxSpeed);

        // Compute the derivative of the position and orientation of the robot:
        float robotAngle = getRobotAngle();
        float xP = velocity * Mathf.Cos(robotAngle);
        float yP = velocity * Mathf.Sin(robotAngle);
        float angleP = velocity * Mathf.Tan(Mathf.Deg2Rad * steering) / L;

        // Update the position and orientation of the robot, given the previous values:
        gameObject.transform.position += h * new Vector3(xP, 0, yP);
        gameObject.transform.Rotate(Vector3.up, h * angleP);
    }

    public float getRobotX() {
        return gameObject.transform.position.x;
    }

    public float getRobotY() {
        return gameObject.transform.position.z;
    }

    public float getRobotAngle() {
        return Mathf.Deg2Rad * (90 - gameObject.transform.rotation.eulerAngles.y);
    }

    // Process a new observation from the LIDAR, and update the algorithm accordingly:
    public void processObservation(float rf, float thetaf) {
        State state_estimate = new State(0, 0, 0);
        float dmin = 1;

        // Covariance matrix of the vehicle location estimate, extracted from P(k|k):
        Matrix<double> Pv = Pkk.Pvv;    // M(dim(state), dim(state))

        float xk = state_estimate.x, yk = state_estimate.y, phik = state_estimate.phi;
        
        // Compute pf, the position of the landmark possibly responsible for this observation:
        Vector<double> pf = g(xk, yk, phik, rf, thetaf);    // M(dim(landmark), 1)

        // Compute Pf, the covariance matrix of pf:
        Matrix<double> gradGxyp = computeGradGxyp(phik, rf, thetaf);    // M(dim(landmark), dim(state))
        Matrix<double> gradGrt = computeGradGrt(phik, rf, thetaf);      // M(dim(landmark), dim(observation))

        // Pv € M(dim(state), dim(state)) and R € M(dim(observation), dim(observation))
        // So Pf € M(dim(landmark), dim(landmark)):
        Matrix<double> Pf = gradGxyp * Pv.TransposeAndMultiply(gradGxyp) 
                        + gradGrt * R.TransposeAndMultiply(gradGrt);

        int acceptedLandmark = -1;
        bool rejectObservation = false;
        for(int i = 0; i < confirmedLandmarks.Count; i++) {
            Vector<double> X = pf - confirmedLandmarks[i].position; // X = pf - pi € M(dim(landmark), 1)
            Matrix<double> Pi = Pkk.extractLandmarkCovariance(i);   // M(dim(landmark), dim(landmark))

            float dfi = (float) (X.ToRowMatrix() * ((Pf + Pi).Inverse()) * X)[0];

            // If dfi < dmin, then the current landmark is chosen as a candidate for the observation:
            if(dfi < dmin) {

                // If no other landmark was accepted before, then accept this landmark:
                if (acceptedLandmark == -1) acceptedLandmark = i;

                // Else if more than one landmark is accepted for this observation, reject the observation:
                else {
                    rejectObservation = true;
                    break;
                }
            }
        }

        // If a confirmed landmark was accepted for the observation, use it to generate a new state estimate:
        if(acceptedLandmark != -1)
            updateStateEstimate(new Observation(rf, thetaf), confirmedLandmarks[acceptedLandmark]);

        // Else if the observation was not rejected, we have to check it against the set of potential landmarks:
        else if(!rejectObservation) {
            // TODO...
        }
    }

    public Vector<double> g(float x, float y, float phi, float rf, float thetaf) {
        float cosphi = Mathf.Cos(phi);
        float sinphi = Mathf.Sin(phi);

        return V.Dense(new double[] {
            x + lidar.a * cosphi - lidar.b * sinphi + rf * Mathf.Cos(phi + thetaf),
            y + lidar.a * sinphi + lidar.b * cosphi + rf * Mathf.Sin(phi + thetaf)});
    }

    // Compute the gradient of g, relatively to x, y and phi:
    public Matrix<double> computeGradGxyp(float phi, float rf, float thetaf) {
        float cosphi = Mathf.Cos(phi);
        float sinphi = Mathf.Sin(phi);

        float dGx_dphi = -lidar.a * sinphi - lidar.b * cosphi - rf * Mathf.Sin(phi + thetaf);
        float dGy_dphi = lidar.a * cosphi - lidar.b * sinphi + rf * Mathf.Cos(phi + thetaf);

        return DenseMatrix.OfArray(new double[,] { 
            { 1, 0, dGx_dphi}, 
            { 0, 1, dGy_dphi } });
    }

    // Compute the gradient of g, relatively to rf and thetaf:
    public Matrix<double> computeGradGrt(float phi, float rf, float thetaf) {
        float cosphi_thetaf = Mathf.Cos(phi + thetaf);
        float sinphi_thetaf = Mathf.Sin(phi + thetaf);

        return DenseMatrix.OfArray(new double[,] {
            { cosphi_thetaf, -rf * sinphi_thetaf },
            { sinphi_thetaf, rf * cosphi_thetaf } });
    }

    // Given the observation from the sensor, and the landmark we associated to this observation, update the state
    // estimate of the robot:
    public void updateStateEstimate(Observation observation, int landmarkIndex) {
        float deltaT = 0; // currentTime - lastTimeUpdate;

        // We start from the previous state estimate, the current control inputs
        // and the previous estimate of the covariance matrix:
        State previousStateEstimate = new State(0, 0, 0);       // x_hat(k|k)
        Inputs inputs = new Inputs(0, 0);                       // Current inputs for the robot
        StateCovariance Pkk = new StateCovariance(0);           // P(k|k)


        Landmark associatedLandmark = confirmedLandmarks[landmarkIndex];

        //// 1. Prediction:
        
        // Equation (10): From the previous state estimate and current inputs,
        // predict a new state estimate: x_hat(k+1|k)
        State statePrediction = predictStateEstimate(previousStateEstimate, inputs, deltaT);

        // Equation (11): From the state prediction, and the landmark associated with the observation,
        // predict what the observation should be: z_hat(k+1|k)
        Observation observationPrediction = predictObservation(statePrediction, associatedLandmark);

        // Equation (12): From the previous state estimate and the current inputs, compute the new estimate of the
        // state covariance matrix: P(k+1|k)
        Matrix<double> Fv = computeFv(previousStateEstimate, inputs, deltaT);
        StateCovariance covariancePrediction = Pkk.predictStateEstimateCovariance(Fv, Q);

        //// 2. Observation:
        float innovation_r = observation.r - observationPrediction.r;
        float innovation_theta = observation.theta - observationPrediction.theta;

        Matrix<double> Hv, Hpi, Si, Wi;
        (Hv, Hpi) = computeHi(statePrediction, associatedLandmark);                                     // Equation (37)
        (Si, Wi) = covariancePrediction.computeInnovationAndGainMatrices(landmarkIndex, Hv, Hpi, R);    // Equation (14)

        //// 3. Update:

    }

    /** Given the previous state estimate and the current inputs, compute the prediction 
     * of the new state estimate according to equation (10) */
    public State predictStateEstimate(State previous, Inputs inputs, float deltaT) {
        float x = previous.x + deltaT * inputs.V * Mathf.Cos(previous.phi);
        float y = previous.y + deltaT * inputs.V * Mathf.Sin(previous.phi);
        float phi = previous.phi + deltaT * inputs.V * Mathf.Tan(inputs.gamma) / L;

        return new State(x, y, phi);
    }

    /* Given the estimated position and the landmark associated to the observation, predict what the observation
     * should be, according to equations (11) and (37) */
    public Observation predictObservation(State predictedState, Landmark landmark) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = lidar.a, b = lidar.b;

        float xr = predictedState.x + a * cosphi - b * sinphi;
        float yr = predictedState.y + a * sinphi + b * cosphi;

        float dX = landmark.x() - xr;
        float dY = landmark.y() - yr;

        float ri = Mathf.Sqrt(dX*dX + dY*dY);
        float thetai = Mathf.Atan(dY / dX) - predictedState.phi;

        return new Observation(ri, thetai);
    }

    /* Given the previous state estimate and the current inputs, compute the Jacobian of f(): Fv */
    public Matrix<double> computeFv(State previous, Inputs inputs, float deltaT) {
        float cosphi = Mathf.Cos(previous.phi);
        float sinphi = Mathf.Sin(previous.phi);

        return DenseMatrix.OfArray(new double[,] {
            {1, 0, -deltaT*inputs.V*sinphi},            // Derivative f1(x, y, phi) / dx
            {0, 1, deltaT*inputs.V*cosphi},             // Derivative f2(x, y, phi) / dy
            {0, 0, 1 } });                              // Derivative f3(x, y, phi) / dphi
    }

    public (Matrix<double>, Matrix<double>) computeHi(State predictedState, Landmark landmark) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = lidar.a, b = lidar.b;

        float xr = predictedState.x + a * cosphi - b * sinphi;
        float yr = predictedState.y + a * sinphi + b * cosphi;

        float dX = landmark.x() - xr, dY = landmark.y() - yr;
        float dX2 = dX * dX, dY2 = dY * dY;
        float sqrt = Mathf.Sqrt(dX2 + dY2);

        // First row of the matrix:
        float A = -dX / sqrt;
        float B = -dY / sqrt;
        float C = (dX * (a * sinphi + b * cosphi) + dY * (b * sinphi - a * cosphi)) / sqrt;

        // Second row of the matrix:
        float D = dY / (dX2 + dY2);
        float E = -1 / (dX * (1 + dY2/dX2));
        float F = ((-(a * cosphi - b * sinphi) * dX - dY * (a * sinphi + b * cosphi)) / (dX2 + dY2)) - 1;

        // Hv = Matrix(OBSERVATION_DIM, STATE_DIM):
        // (    dr/dx,      dr/dy,      dr/dphi)
        // (dtheta/dx,  dtheta/dy,  dtheta/dphi)
        Matrix<double> Hv = DenseMatrix.OfArray(new double[,] { { A, B, C }, { D, E, F } });

        // Hpi = Matrix(OBSERVATION_DIM, LANDMARK_DIM):
        // (    dr/dxi,     dr/dyi)
        // (dtheta/dxi, dtheta/dyi)
        Matrix<double> Hpi = DenseMatrix.OfArray(new double[,] { { -A, -B }, { -D, -E } });

        return (Hv, Hpi);
    }
}
