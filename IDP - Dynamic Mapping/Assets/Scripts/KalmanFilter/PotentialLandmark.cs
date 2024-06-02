using MathNet.Numerics.LinearAlgebra;

public class PotentialLandmark {
    private Vector<double> position;
    private Matrix<double> covariance;
    private long creationTimestep;
    private int countAssociations;

    public PotentialLandmark(Vector<double> position, Matrix<double> covariance, long timestep) {
        this.position = position;
        this.covariance = covariance;
        this.creationTimestep = timestep;
        this.countAssociations = 0;
    }

    public void updateState(Vector<double> newPosEstimate, Matrix<double> newPosCovariance) {
        // We update the landmark covariance and position estimates using the Kalman gain:
        Matrix<double> K = covariance * (covariance + newPosCovariance).Inverse();

        position = position + K * (newPosEstimate - position);
        covariance = covariance - K * covariance;
        
        countAssociations++;
    }

    public Landmark toLandmark() {
        return new Landmark(position);
    }

    public Vector<double> getPosition() {
        return position;
    }

    public Matrix<double> getCovariance() {
        return covariance;
    }

    public long getCreationTime() {
        return creationTimestep;
    }

    public int getAssociationsCount() {
        return countAssociations;
    }
}
