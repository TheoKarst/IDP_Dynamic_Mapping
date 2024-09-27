using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Class used to represent the state covariance matrix of the robot. According to the implementation
/// in "A Solution to the Simultaneous Localization and Map Building(SLAM) Problem", the state to estimate
/// shouldn't be just the vehicle state (x, y, phi), but also the position of the landmarks.
/// This is why this class is used to represent the cross-correlation between landmarks, and requires
/// a big resizeable matrix with the size: (STATE_DIM + N*LANDMARKS_DIM) x (STATE_DIM + N*LANDMARKS_DIM)
/// Where:
///     STATE_DIM = 3 is the dimension of the vehicle state (x, y, phi)
///     LANDMARKS_DIM = 2 is the dimension of the landmarks (x, y)
///     N is the number of confirmed landmarks
/// </summary>

// Create a class to represent P(i|j), and the operations related to it:
public class StateCovariance {
    /* P(i|j) = [ Pvv(i|j),              Pvm(i|j) ]
     *          [ Pvm(i|j).Transpose(),  Pmm(i|j) ]
     * 
     * Pvv(i|j): error covariance matrix associated with the vehicle state estimate
     * Pmm(i|j): map covariance matrix associated with the landmarks state estimates
     * Pvm(i|j): cross-covariance matrix between vehicle and landmark states
     */

    private static MatrixBuilder<double> M = Matrix<double>.Build;

    private Matrix<double> P;
    private static int STATE_DIM;
    private static int LANDMARK_DIM;

    /// <summary>
    /// Instantiates the StateCovariance matrix as a zero matrix of size SxS
    /// where S = stateDim + landmarksCount * landmarkDim
    /// </summary>
    /// <param name="stateDim"></param>
    /// <param name="landmarksCount"></param>
    /// <param name="landmarkDim"></param>
    public StateCovariance(int stateDim, int landmarksCount, int landmarkDim) {
        STATE_DIM = stateDim;
        LANDMARK_DIM = landmarkDim;

        int size = STATE_DIM + landmarksCount * LANDMARK_DIM;
        P = M.DenseDiagonal(size, size, 0);
    }

    /// <summary>
    /// Instantiates the StateCovariance from a given matrix
    /// </summary>
    private StateCovariance(Matrix<double> P) {
        this.P = P;
    }

    /// <summary>
    /// Instantiates the StateCovariance matrix from it's sub matrices
    /// </summary>
    /// <param name="Pvv">Error covariance matrix associated with the vehicle state estimate</param>
    /// <param name="Pvm">Cross-covariance matrix between vehicle and landmark states</param>
    /// <param name="Pmm">Covariance matrix associated with the landmarks state estimates</param>
    private StateCovariance(Matrix<double> Pvv, Matrix<double> Pvm, Matrix<double> Pmm) {
        P = M.Dense(Pvv.RowCount + Pmm.RowCount, Pvv.ColumnCount + Pmm.ColumnCount);
        P.SetSubMatrix(0, 0, Pvv);
        P.SetSubMatrix(0, STATE_DIM, Pvm);
        P.SetSubMatrix(STATE_DIM, 0, Pvm.Transpose());
        P.SetSubMatrix(STATE_DIM, STATE_DIM, Pmm);
    }

    /// <summary>
    /// Returns the covariance matrix associated with the vehicle state estimate
    /// </summary>
    public Matrix<double> ExtractPvv() {
        return P.SubMatrix(0, STATE_DIM, 0, STATE_DIM);
    }

    /// <summary>
    /// Returns the covariance matrix of the given landmark
    /// </summary>
    public Matrix<double> ExtractLandmarkCovariance(int landmarkIndex) {
        int index = STATE_DIM + landmarkIndex * LANDMARK_DIM;
        return P.SubMatrix(index, LANDMARK_DIM, index, LANDMARK_DIM);
    }

    /// <summary>
    /// Extend this matrix to insert the covariance of the given landmark
    /// </summary>
    public void AddLandmark(Matrix<double> landmarkCovariance) {
        Matrix<double> newP = M.Dense(P.RowCount+LANDMARK_DIM, P.ColumnCount+LANDMARK_DIM);
        newP.SetSubMatrix(0, 0, P);
        newP.SetSubMatrix(P.RowCount, P.ColumnCount, landmarkCovariance);

        P = newP;
    }

    /// <summary>
    /// Performs the prediction step of the Kalman Filter, to predict the new state covariance matrix
    /// from the state transition matrix
    /// </summary>
    /// <param name="Fv">State transition matrix of the vehicle (3x3 matrix)</param>
    /// <param name="model"></param>
    /// <returns></returns>
    public StateCovariance PredictStateEstimateCovariance(Matrix<double> Fv, VehicleModel model) {
        int landmarksSize = P.ColumnCount - STATE_DIM;

        Matrix<double> Pvv = P.SubMatrix(0, STATE_DIM, 0, STATE_DIM);
        Matrix<double> Pvm = P.SubMatrix(0, STATE_DIM, STATE_DIM, landmarksSize);
        Matrix<double> Pmm = P.SubMatrix(STATE_DIM, landmarksSize, STATE_DIM, landmarksSize);

        // Computes P = F.dot(P).dot(F.transpose()) + Q efficiently, since F is almost the identity matrix
        // (only the top-left 3x3 bloc of the matrix is Fv and not the identity):
        return new StateCovariance(
            Fv * Pvv.TransposeAndMultiply(Fv) + model.ProcessNoiseError,
            Fv * Pvm,
            Pmm);
    }

    /// <summary>
    /// Computes the innovation and gain matrices from the observation matrix and the observation error matrix
    /// </summary>
    /// <param name="H">Observation matrix: OBSERVATION_DIM x (STATE_DIM+landmarkCount*LANDMARK_DIM)</param>
    /// <param name="R">Observation error matrix: OBSERVATION_DIM x OBSERVATION_DIM</param>
    /// <returns></returns>
    public (Matrix<double>, Matrix<double>) ComputeInnovationAndGainMatrices(Matrix<double> H, Matrix<double> R) {
        Matrix<double> tmp = P.TransposeAndMultiply(H);

        Matrix<double> Si = H * tmp + R;           // Matrix(OBSERVATION_DIM, OBSERVATION_DIM)
        Matrix<double> Wi = tmp * Si.Inverse();    // Matrix(STATE_DIM+landmarkCount*LANDMARK_DIM, OBSERVATION_DIM)

        return (Si, Wi);
    }

    public static StateCovariance operator +(StateCovariance state, Matrix<double> other) {
        return new StateCovariance(state.P + other);
    }

    public static StateCovariance operator -(StateCovariance state, Matrix<double> other) {
        return new StateCovariance(state.P - other);
    }
}