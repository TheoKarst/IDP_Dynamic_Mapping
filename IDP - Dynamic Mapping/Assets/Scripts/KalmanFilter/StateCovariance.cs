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

    private static MatrixBuilder<double> M = Matrix<double>.Build;

    private Matrix<double> P;
    private static int STATE_DIM;
    private static int LANDMARK_DIM;

    public StateCovariance(int stateDim, int landmarksCount, int landmarkDim) {
        STATE_DIM = stateDim;
        LANDMARK_DIM = landmarkDim;

        int size = STATE_DIM + landmarksCount * LANDMARK_DIM;
        P = M.DenseDiagonal(size, size, 0);
    }

    private StateCovariance(Matrix<double> P) {
        this.P = P;
    }

    private StateCovariance(Matrix<double> Pvv, Matrix<double> Pvm, Matrix<double> Pmm) {
        P = M.Dense(Pvv.RowCount + Pmm.RowCount, Pvv.ColumnCount + Pmm.ColumnCount);
        P.SetSubMatrix(0, 0, Pvv);
        P.SetSubMatrix(0, STATE_DIM, Pvm);
        P.SetSubMatrix(STATE_DIM, 0, Pvm.Transpose());
        P.SetSubMatrix(STATE_DIM, STATE_DIM, Pmm);
    }

    public Matrix<double> ExtractPvv() {
        return P.SubMatrix(0, STATE_DIM, 0, STATE_DIM);
    }

    public Matrix<double> ExtractLandmarkCovariance(int landmarkIndex) {
        int index = STATE_DIM + landmarkIndex * LANDMARK_DIM;
        return P.SubMatrix(index, LANDMARK_DIM, index, LANDMARK_DIM);
    }

    public void AddLandmark(Matrix<double> landmarkCovariance) {
        Matrix<double> newP = M.Dense(P.RowCount+LANDMARK_DIM, P.ColumnCount+LANDMARK_DIM);
        newP.SetSubMatrix(0, 0, P);
        newP.SetSubMatrix(P.RowCount, P.ColumnCount, landmarkCovariance);

        P = newP;
    }

    public StateCovariance PredictStateEstimateCovariance(Matrix<double> Fv, VehicleModel model) {
        int landmarksSize = P.ColumnCount - STATE_DIM;

        Matrix<double> Pvv = P.SubMatrix(0, STATE_DIM, 0, STATE_DIM);
        Matrix<double> Pvm = P.SubMatrix(0, STATE_DIM, STATE_DIM, landmarksSize);
        Matrix<double> Pmm = P.SubMatrix(STATE_DIM, landmarksSize, STATE_DIM, landmarksSize);

        return new StateCovariance(
            Fv * Pvv.TransposeAndMultiply(Fv) + model.ProcessNoiseError,
            Fv * Pvm,
            Pmm);
    }

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