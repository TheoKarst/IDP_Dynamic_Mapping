using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Class used to represent a potential landmark in the Kalman Filter
/// </summary>

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

    public void UpdateState(Vector<double> newPosEstimate, Matrix<double> newPosCovariance) {
        // We update the landmark covariance and position estimates using the Kalman gain:
        Matrix<double> K = covariance * (covariance + newPosCovariance).Inverse();

        position = position + K * (newPosEstimate - position);
        covariance = covariance - K * covariance;
        
        countAssociations++;
    }

    public Landmark ToLandmark() {
        return new Landmark(position);
    }

    public Vector<double> GetPosition() {
        return position;
    }

    public Matrix<double> GetCovariance() {
        return covariance;
    }

    public long GetCreationTime() {
        return creationTimestep;
    }

    public int GetAssociationsCount() {
        return countAssociations;
    }
}
