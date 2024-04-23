using MathNet.Numerics.LinearAlgebra;

// Create a class to represent P(i|j), and the operations related to it:
public class StateCovariance {
    /* P(i|j) = [ Pvv(i|j),              Pvm(i|j) ]
     *          [ Pvm(i|j).Transpose(),  Pmm(i|j) ]
     * 
     * Pvv(i|j): error covariance matrix associated with the vehicle state estimate
     * Pmm(i|j): map covariance matrix associated with the landmarks state estimates
     * Pvm(i|j): cross-covariance matrix between vehicle and landmark states
     */

    private static MatrixBuilder<float> M = Matrix<float>.Build;

    private Matrix<float> P;
    private static int STATE_DIM;
    private static int LANDMARK_DIM;

    public StateCovariance(int stateDim, int landmarksCount, int landmarkDim) {
        STATE_DIM = stateDim;
        LANDMARK_DIM = landmarkDim;

        P = M.DenseIdentity(STATE_DIM + landmarksCount * LANDMARK_DIM);
    }

    private StateCovariance(Matrix<float> P) {
        this.P = P;
    }

    private StateCovariance(Matrix<float> Pvv, Matrix<float> Pvm, Matrix<float> Pmm) {
        P = M.Dense(Pvv.RowCount + Pmm.RowCount, Pvv.ColumnCount + Pmm.ColumnCount);
        P.SetSubMatrix(0, 0, Pvv);
        P.SetSubMatrix(0, STATE_DIM, Pvm);
        P.SetSubMatrix(STATE_DIM, 0, Pvm.Transpose());
        P.SetSubMatrix(STATE_DIM, STATE_DIM, Pmm);
    }

    public Matrix<float> extractPvv() {
        return P.SubMatrix(0, STATE_DIM, 0, STATE_DIM);
    }

    public Matrix<float> extractLandmarkCovariance(int landmarkIndex) {
        int index = STATE_DIM + landmarkIndex * LANDMARK_DIM;
        return P.SubMatrix(index, LANDMARK_DIM, index, LANDMARK_DIM);
    }

    public StateCovariance predictStateEstimateCovariance(Matrix<float> Fv, Matrix<float> Q) {
        int landmarksSize = P.ColumnCount - STATE_DIM;

        Matrix<float> Pvv = P.SubMatrix(0, STATE_DIM, 0, STATE_DIM);
        Matrix<float> Pvm = P.SubMatrix(0, STATE_DIM, STATE_DIM, landmarksSize);
        Matrix<float> Pmm = P.SubMatrix(STATE_DIM, landmarksSize, STATE_DIM, landmarksSize);

        return new StateCovariance(
            Fv * Pvv.TransposeAndMultiply(Fv) + Q,
            Fv * Pvm,
            Pmm);
    }

    public (Matrix<float>, Matrix<float>) computeInnovationAndGainMatrices(Matrix<float> Hi, Matrix<float> R) {
        Matrix<float> tmp = P.TransposeAndMultiply(Hi);

        Matrix<float> Si = Hi * tmp + R;           // Matrix(OBSERVATION_DIM, OBSERVATION_DIM)
        Matrix<float> Wi = tmp * Si.Inverse();     // Matrix(STATE_DIM+landmarkCount*LANDMARK_DIM, OBSERVATION_DIM)

        return (Si, Wi);
    }

    public static StateCovariance operator +(StateCovariance state, Matrix<float> other) {
        return new StateCovariance(state.P + other);
    }

    public static StateCovariance operator -(StateCovariance state, Matrix<float> other) {
        return new StateCovariance(state.P - other);
    }
}