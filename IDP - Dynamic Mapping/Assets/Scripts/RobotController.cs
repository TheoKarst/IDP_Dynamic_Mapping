using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;

public class RobotController : MonoBehaviour
{
    public struct Landmark {
        public float x;
        public float y;
        public Landmark(float x, float y) {
            this.x = x;
            this.y = y;
        }
    }

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

    public struct Observation {
        public float r;
        public float theta;

        public Observation(float r, float theta) {
            this.r = r;
            this.theta = theta;
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

    // Matrix builder used as a shortcut for matrix creation:
    private MatrixBuilder<double> M = Matrix<double>.Build;
    private Matrix<double> Q, R;

    // Start is called before the first frame update
    void Start() {
        Q = M.DenseIdentity(3);         // Covariance matrix for the process noise errors (x, y, phi)
        R = M.DenseIdentity(2);         // Covariance matrix for the observation errors (r, theta)
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
    public void processObservation(Observation observation) {
        State state_estimate = new State(0, 0, 0);

        // Covariance matrix of the vehicle location estimate, extracted from P(k|k):
        Matrix<double> Pv = M.DenseIdentity(3);

        float xk = state_estimate.x;
        float yk = state_estimate.y;
        float phik = state_estimate.phi;

        float rf = observation.r;
        float thetaf = observation.theta;

        float cosphi = Mathf.Cos(phik);
        float sinphi = Mathf.Sin(phik);

        // Compute pf, the position of the landmark possibly responsible for this observation.
        // pf = (xf, yf) = g(xk, yk, phik, rf, thetaf):
        float xf = xk + lidar.a * cosphi - lidar.b * sinphi + rf * Mathf.Cos(phik + thetaf);
        float yf = yk + lidar.a * sinphi + lidar.b * cosphi + rf * Mathf.Sin(phik + thetaf);

        // Compute Pf, the covariance matrix of pf:
        Matrix<double> gradGxyp = computeGradGxyp(phik, rf, thetaf);
        Matrix<double> gradGrt = computeGradGrt(phik, rf, thetaf);

        Matrix<double> Pf = gradGxyp * Pv * gradGxyp.Transpose() + gradGrt * R * gradGrt.Transpose();

        // TODO: Compute dfi, compare it to dmin...
        // ...

        // Get the landmark associated to the observation:
        Landmark associatedLandmark = confirmedLandmarks[0];

        // Finally, if the observation is recognized as a landmark, we can use it to perform a new state estimate:
        updateStateEstimate(observation, associatedLandmark);
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
    public void updateStateEstimate(Observation observation, Landmark associatedLandmark) {
        float deltaT = 0; // currentTime - lastTimeUpdate;

        // We start from a current state estimate, and control inputs:
        State state_estimate = new State(0, 0, 0);      // x_hat(k|k)
        float V = 0;
        float gamma = 0;


        // 1. Prediction:
        State state_prediction = predictStateEstimate(state_estimate, V, gamma, deltaT);                // Equation (10)
        Observation observation_prediction = predictObservation(state_prediction, associatedLandmark);  // Equation (11)
        Matrix<double> covariance_prediction = predictStateEstimateCovariance();                        // Equation (12)

        // 2. Observation:
        float innovation_r = observation.r - observation_prediction.r;
        float innovation_theta = observation.theta - observation_prediction.theta;

        Matrix<double> Hi = computeHi(state_prediction, associatedLandmark);    // Equation (37)
        Matrix<double> Si = Hi * covariance_prediction * Hi.Transpose() + R;    // Equation (14)

        // 3. Update:

    }

    /** Given the previous state estimate, compute the prediction of the new state estimate 
     * according to equation (10) */
    public State predictStateEstimate(State previous, float V, float gamma, float deltaT) {
        float x = previous.x + deltaT * V * Mathf.Cos(previous.phi);
        float y = previous.y + deltaT * V * Mathf.Sin(previous.phi);
        float phi = previous.phi + deltaT * V * Mathf.Tan(gamma) / L;

        return new State(x, y, phi);
    }

    /* Given the estimated position and the landmark associated to the observation, predict what the observation
     * should be, according to equations (11) and (37) */
    public Observation predictObservation(State predictedState, Landmark landmark) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = lidar.a, b = lidar.b;

        float xr = predictedState.x + a * cosphi - b * sinphi;
        float yr = predictedState.y + a * sinphi + b * cosphi;

        float dX = landmark.x - xr;
        float dY = landmark.y - yr;

        float ri = Mathf.Sqrt(dX*dX + dY*dY);
        float thetai = Mathf.Atan(dY / dX) - predictedState.phi;

        return new Observation(ri, thetai);
    }

    public Matrix<double> predictStateEstimateCovariance() {
        return null;
    }

    public Matrix<double> computeF(float V, float phi, float gamma, float deltaT, float L) {
        float sinphi = Mathf.Sin(phi);
        float cosphi = Mathf.Cos(phi);
        float tangamma = Mathf.Tan(gamma);

        return DenseMatrix.OfArray(new double[,] {
            {1, 0, -deltaT*V*sinphi, deltaT*cosphi, 0},
            {0, 1, deltaT*V*cosphi, deltaT*sinphi, 0},
            {0, 0, 1, deltaT*tangamma/L, deltaT*V*(1+tangamma*tangamma)/L } });
    }

    public Matrix<double> computeHi(State predictedState, Landmark landmark) {
        float cosphi = Mathf.Cos(predictedState.phi), sinphi = Mathf.Sin(predictedState.phi);
        float a = lidar.a, b = lidar.b;

        float xr = predictedState.x + a * cosphi - b * sinphi;
        float yr = predictedState.y + a * sinphi + b * cosphi;

        float dX = xr - landmark.x, dY = yr - landmark.y;
        float dX2 = dX * dX, dY2 = dY * dY;

        // First row of the matrix:
        float tmp = Mathf.Sqrt(dX2 + dY2);

        float A = dX / tmp;
        float B = dY / tmp;
        float C = (dX * (-a * sinphi - b * cosphi) + dY * (a * cosphi - b * sinphi)) / tmp;

        // Second row of the matrix:
        tmp = 1 + dY2 / dX2;

        float D = -dY / (dX2 * tmp);
        float E = 1 / (dX * tmp);
        float F = (a * cosphi - b * sinphi) * dX + dY * (a * sinphi + b * cosphi);
        F = F / (dX2 * tmp) - 1;

        return DenseMatrix.OfArray(new double[,] {{A, B, C}, {D, E, F}});
    }
}
