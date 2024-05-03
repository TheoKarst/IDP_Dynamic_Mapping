using MathNet.Numerics.LinearAlgebra;

public class PotentialLandmark {
    private Vector<float> position;
    private Matrix<float> covariance;
    private long creationTimestep;
    private int countAssociations;

    public PotentialLandmark(Vector<float> position, Matrix<float> covariance, long timestep) {
        this.position = position;
        this.covariance = covariance;
        this.creationTimestep = timestep;
        this.countAssociations = 0;
    }

    public void updateState(Vector<float> newPosEstimate, Matrix<float> newPosCovariance) {
        // We update the landmark covariance and position estimates using the Kalman gain:
        Matrix<float> K = covariance * (covariance + newPosCovariance).Inverse();

        position = position + K * (newPosEstimate - position);
        covariance = covariance - K * covariance;
        
        countAssociations++;
    }

    public Landmark toLandmark() {
        return new Landmark(position);
    }

    public Vector<float> getPosition() {
        return position;
    }

    public Matrix<float> getCovariance() {
        return covariance;
    }

    public long getCreationTime() {
        return creationTimestep;
    }

    public int getAssociationsCount() {
        return countAssociations;
    }
}
