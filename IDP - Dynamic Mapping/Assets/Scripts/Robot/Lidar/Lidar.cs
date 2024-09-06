using System.Collections.Generic;
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
    private List<Observation> observations;

    // Used for drawing only:
    private Vector3 lastScanPosition, lastScanForward;  // Real position of the LIDAR during last scan

    public Lidar(GameObject lidar, int raycastCount, float minRange, float maxRange, int lidarIndex) {
        this.lidar = lidar;
        this.lidarIndex = lidarIndex;
        this.raycastCount = raycastCount;
        this.minRange = minRange;
        this.maxRange = maxRange;
    }

    public void DrawGizmos(bool drawRays) {
        if (drawRays && observations != null) {
            Vector3 direction = lastScanForward;

            foreach (Observation observation in observations) {
                Gizmos.color = Color.red;
                Gizmos.DrawRay(lastScanPosition, direction * observation.r);

                direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            }
        }
    }

    // Use raycasting to compute the observations made by the LIDAR. Observations with a range
    // smaller than minRange are ignored and observations with a range larger than maxRange are
    // clamped and marked as outOfRange:
    public Observation[] ComputeObservations() {
        observations = new List<Observation>(raycastCount);

        // Save the current position and orientation of the LIDAR, to draw gizmos later:
        lastScanPosition = lidar.transform.position;
        lastScanForward = lidar.transform.TransformDirection(Vector3.forward);

        // Use raycasting to compute the current observations:
        float observationAngle = 0;
        Vector3 direction = lidar.transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < raycastCount; i++) {
            RaycastHit hit;
            if (Physics.Raycast(lidar.transform.position, direction, out hit, maxRange + 1)
                && hit.distance <= maxRange) {

                // The observation is added only if the range is bigger than minRange:
                if(hit.distance >= minRange)
                    observations.Add(new Observation(hit.distance, observationAngle, lidarIndex, false));
            }
            else {
                // If there was no hit, the range is clamped at maxRange, and the observation is
                // marked as out of range:
                observations.Add(new Observation(maxRange, observationAngle, lidarIndex, true));
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            observationAngle += 2 * Mathf.PI / raycastCount;
        }

        return observations.ToArray();
    }

    // Local pose of the LIDAR on the robot:
    public Pose2D GetLocalPose() {
        float x = lidar.transform.localPosition.z;
        float y = -lidar.transform.localPosition.x;
        float angle = -lidar.transform.localRotation.eulerAngles.y * Mathf.Deg2Rad;

        return new Pose2D(x, y, angle);
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
