using UnityEngine;

public class Lidar {

    // GameObject representing the LIDAR:
    private GameObject lidar;

    // Number of raycasts produced by the LIDAR:
    private int raycastCount;

    // Maximum distance of the raycasts:
    private float raycastDistance;

    // List of observations made by the LIDAR:
    private AugmentedObservation[] observations;

    // Used for drawing only:
    private Vector3 lastScanPosition, lastScanForward;  // Real position of the LIDAR during last scan

    public Lidar(GameObject lidar, int raycastCount, float raycastDistance) {
        this.lidar = lidar;
        this.raycastCount = raycastCount;
        this.raycastDistance = raycastDistance;

        this.observations = new AugmentedObservation[raycastCount];
    }

    public void DrawGizmos(bool drawRays) {
        if (drawRays && observations != null && observations[0] != null) {
            Vector3 direction = lastScanForward;

            foreach (AugmentedObservation observation in observations) {
                Gizmos.color = observation.outOfRange ? Color.white : Color.red;
                Gizmos.DrawRay(lastScanPosition, direction * observation.r);

                direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            }
        }
    }

    // Use raycasting to compute the observations made by the LIDAR:
    public AugmentedObservation[] ComputeObservations() {
        // Save the current position and orientation of the LIDAR, to draw gizmos later:
        lastScanPosition = lidar.transform.position;
        lastScanForward = lidar.transform.TransformDirection(Vector3.forward);

        // Use raycasting to compute the current observations:
        float observationAngle = 0;
        Vector3 direction = lidar.transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < observations.Length; i++) {
            RaycastHit hit;
            if (Physics.Raycast(lidar.transform.position, direction, out hit, raycastDistance)) {
                observations[i] = new AugmentedObservation(hit.distance, observationAngle, false);
            }
            else {
                observations[i] = new AugmentedObservation(raycastDistance, observationAngle, true);
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            observationAngle += 2 * Mathf.PI / raycastCount;
        }

        return observations;
    }

    /*
    public WipeShape BuildWipeShape(RobotManager manager, VehicleState vehicleState, Vector2 sensorPosition, float extentMargin) {
        
        // Use the filteredCorners to build the WipeShape:
        List<Point> shapePoints = new List<Point>(filteredCorners.Count);
        foreach (int index in filteredCorners) {
            ExtendedObservation observation = observations[index];

            // Add a margin to the observation:
            observation.r += extentMargin;

            // Compute the world space position of the observation:
            Vector<double> position = manager.ComputeObservationPositionEstimate(observation.ToObservation());
            float x = (float)position[0], y = (float)position[1];

            // Compute the world space angle of the observation:
            float angle = vehicleState.phi + observation.theta;

            // Build the point:
            shapePoints.Add(new Point(x, y, angle));
        }

        return new WipeShape(sensorPosition, shapePoints);
    }*/

    // Position of the LIDAR on the robot:
    public (float, float) GetLocalPosition() {
        return (lidar.transform.localPosition.z, -lidar.transform.localPosition.x);
    }

    public AugmentedObservation[] GetObservations() {
        return observations;
    }

    public void DrawObservation(Observation observation, Color color) {
        float duration = 0.02f;

        Vector3 direction = lidar.transform.TransformDirection(Vector3.forward);
        direction = Quaternion.AngleAxis(-observation.theta * Mathf.Rad2Deg, Vector3.up) * direction;

        Debug.DrawRay(lidar.transform.position, direction * observation.r, color, duration);
    }    
}
