public class ModelParams {
    // Dimensions of the model:
    public float L;         // Length of the vehicle
    public float a, b;      // Parameters defining the local position of the LIDAR

    // Error estimates of the model:
    public float errorX;
    public float errorY;
    public float errorPhi;
    public float errorR;
    public float errorTheta;

    // Minimal Euclidean distance between confirmed landmarks:
    public float minDistanceBetweenLandmarks;
}