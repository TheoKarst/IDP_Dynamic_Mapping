using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Class used to represent a frame of data in the simulation. Each robot should be able to
/// produce this kind of frames, to be compatible with the mapping algorithms
/// </summary>

public class RobotData {
    public float timestamp;

    public VehicleState vehicleState;
    public Matrix<double> vehicleStateCovariance;

    // List of observations made by each LIDAR on the robot:
    public Observation[][] observations;

    public RobotData(float timestamp, VehicleState state,
        Matrix<double> stateCovariance, Observation[][] observations) {

        this.timestamp = timestamp;
        this.vehicleState = state;
        this.vehicleStateCovariance = stateCovariance;
        this.observations = observations;
    }
}