using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour
{
    public int raycastCount = 20;
    public float raycastDistance = 1;

    // For each measurement i of the LIDAR, we check if it's a corner by comparing it with the measure
    // i + deltaCorners and the measure i - deltaCorners:
    public int deltaCorners = 1;

    // The minimum angle for a corner to be detected as such:
    public float angleThreshold = 80;

    public bool drawRays = true;
    public bool drawCorners = true;

    // Real position of the hit points from the LIDAR (only used for drawing):
    private Vector3[] hitPoints;

    // List of observations made by the LIDAR, and indices of observations corresponding to 
    // convex and concave corners:
    private Observation[] observations;
    private List<int> convexCorners, concaveCorners;

    // Start is called before the first frame update
    void Start() {
        hitPoints = new Vector3[raycastCount];
        observations = new Observation[raycastCount];
    }

    // Update is called once per frame
    void Update() {
        // Compute the raycast intersections with the environment, and update the observations:
        ComputeObservations();

        // From the observations, detect the ones that correspond to corners (that are used as landmarks):
        DetectCorners();
    }

    public void OnDrawGizmos() {
        if (drawCorners && convexCorners != null) {
            Gizmos.color = Color.green;

            foreach (int index in convexCorners)
                Gizmos.DrawSphere(hitPoints[index], 0.1f);
        }

        if (drawCorners && concaveCorners != null) {
            Gizmos.color = Color.red;

            foreach (int index in concaveCorners)
                Gizmos.DrawSphere(hitPoints[index], 0.1f);
        }
    }

    // Use raycasting to compute the observations made by the LIDAR:
    private void ComputeObservations() {
        float observationAngle = 0;
        Vector3 direction = transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < observations.Length; i++) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, raycastDistance)) {
                // Used for drawing only:
                if (drawRays) Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                hitPoints[i] = hit.point;

                // Used for localisation, mapping, etc...
                observations[i] = new Observation(hit.distance, observationAngle);
            }
            else {
                // Used for drawing only:
                if (drawRays) Debug.DrawRay(transform.position, direction * raycastDistance, Color.white);
                hitPoints[i] = Vector3.zero;

                // Used for localisation, mapping, etc...
                observations[i] = new Observation(-1, observationAngle);
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            observationAngle += 2 * Mathf.PI / raycastCount;
        }
    }

    private void DetectCorners() {
        if (observations == null)
            return;

        convexCorners = new List<int>();
        concaveCorners = new List<int>();
        
        int count = observations.Length;

        // Use the same naming convention as the paper "Corner Detection for Room Mapping of Fire Fighting Robot":
        for(int i = 0; i < count; i++) {
            Observation prev = observations[(i + count - deltaCorners) % count];
            Observation curr = observations[i];
            Observation next = observations[(i + deltaCorners) % count];

            float b = prev.r, c = curr.r, e = next.r;

            // If any of the measurement found no obstacle, then ignore this point:
            if (b < 0 || c < 0 || e < 0)
                continue;

            // Else, detect if this is a corner using cosine law:
            float a = Mathf.Sqrt(b * b + c * c - 2 * b * c * Mathf.Cos(curr.theta - prev.theta));
            float d = Mathf.Sqrt(c * c + e * e - 2 * c * e * Mathf.Cos(next.theta - curr.theta));

            float angleB = Mathf.Acos((a*a + c*c - b*b) / (2*a*c));
            float angleE = Mathf.Acos((d*d + c*c - e*e) / (2*d*c));

            // Angle in degrees of the corner:
            float theta = Mathf.Rad2Deg * (angleB + angleE);

            if (theta < 140)
                concaveCorners.Add(i);

            else if (theta > 220)
                convexCorners.Add(i);
        }
    }

    public (float, float) GetLocalPosition() {
        return (transform.localPosition.z, -transform.localPosition.x);
    }

    public Observation[] GetObservations() {
        if (observations == null || observations[0] == null)
            return null;

        return observations;
    }

    // Use the convex corners as landmarks to update our state estimate:
    public Observation[] GetLandmarkCandidates() {
        Observation[] landmarks = new Observation[convexCorners.Count];
        
        for (int i = 0; i < landmarks.Length; i++)
            landmarks[i] = observations[convexCorners[i]];

        return landmarks;
    }

    public void DrawObservation(Observation observation, Color color) {
        float duration = 0.1f;

        Vector3 direction = transform.TransformDirection(Vector3.forward);
        direction = Quaternion.AngleAxis(-observation.theta * Mathf.Rad2Deg, Vector3.up) * direction;

        Debug.DrawRay(transform.position, direction * observation.r, color, duration);
    }
}
