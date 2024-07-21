using UnityEngine;

public class Lidar {

    // GameObject representing the LIDAR:
    private GameObject lidar;

    // Index of this LIDAR (since we may want to use multiple LIDARs):
    private int lidarIndex;

    // Number of raycasts produced by the LIDAR:
    private int raycastCount;

    // Minimum valid range of the LIDAR (smaller ranges should be discarded):
    private float minRange;

    // Maximum valid range of the LIDAR (bigger ranges should be clamped):
    private float maxRange;

    // List of observations made by the LIDAR:
    private Observation[] observations;

    // Used for drawing only:
    private Vector3 lastScanPosition, lastScanForward;  // Real position of the LIDAR during last scan

    public Lidar(GameObject lidar, int raycastCount, float minRange, float maxRange, int lidarIndex) {
        this.lidar = lidar;
        this.lidarIndex = lidarIndex;
        this.raycastCount = raycastCount;
        this.minRange = minRange;
        this.maxRange = maxRange;

        this.observations = new Observation[raycastCount];
    }

    public void DrawGizmos(bool drawRays) {
        if (drawRays && observations != null) {
            Vector3 direction = lastScanForward;

            foreach (Observation observation in observations) {
                Gizmos.color = observation.r < minRange || observation.r > maxRange ? 
                    Color.white : Color.red;

                Gizmos.DrawRay(lastScanPosition, direction * observation.r);

                direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            }
        }
    }

    // Use raycasting to compute the observations made by the LIDAR:
    public Observation[] ComputeObservations() {
        // Save the current position and orientation of the LIDAR, to draw gizmos later:
        lastScanPosition = lidar.transform.position;
        lastScanForward = lidar.transform.TransformDirection(Vector3.forward);

        // Use raycasting to compute the current observations:
        float observationAngle = 0;
        Vector3 direction = lidar.transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < observations.Length; i++) {
            RaycastHit hit;
            if (Physics.Raycast(lidar.transform.position, direction, out hit)) {
                observations[i] = new Observation(hit.distance, observationAngle, lidarIndex);
            }
            else {
                // If there was no hit, consider that there was a hit at 2*maxDistance, to represent
                // +infinity. Observations with a range greater than maxRange are treated identically
                // anyways:
                observations[i] = new Observation(2*maxRange, observationAngle, lidarIndex);
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            observationAngle += 2 * Mathf.PI / raycastCount;
        }

        return observations;
    }

    // Local pose of the LIDAR on the robot:
    public Pose2D GetLocalPose() {
        float x = lidar.transform.localPosition.z;
        float y = -lidar.transform.localPosition.x;
        float angle = -lidar.transform.localRotation.eulerAngles.y * Mathf.Deg2Rad;

        return new Pose2D(x, y, angle);
    }

    public Observation[] GetObservations() {
        return observations;
    }

    public LidarSetup GetSetup() {
        return new LidarSetup(lidarIndex, lidar.name, GetLocalPose(), minRange, maxRange);
    }

    public void DrawObservation(Observation observation, Color color) {
        float duration = 0.02f;

        Vector3 direction = lidar.transform.TransformDirection(Vector3.forward);
        direction = Quaternion.AngleAxis(-observation.theta * Mathf.Rad2Deg, Vector3.up) * direction;

        Debug.DrawRay(lidar.transform.position, direction * observation.r, color, duration);
    }    
}
