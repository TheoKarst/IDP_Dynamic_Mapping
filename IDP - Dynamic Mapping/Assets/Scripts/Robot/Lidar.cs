using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour {

    [Tooltip("Model used to represent the world state estimate")]
    public RobotBresenham worldModel;

    [Tooltip("Number of raycasts produced by the LIDAR")]
    public int raycastCount = 500;

    [Tooltip("Maximum distance of the raycasts")]
    public float raycastDistance = 20;

    [Tooltip("Epsilon we are using in Douglas Peucker algorithm to simplify the " +
        "contour shape of the LIDAR, before corners extraction")]
    public float DouglasPeuckerEpsilon = 0.2f;

    public bool drawRays = true;
    public bool drawCorners = true;

    // Real position of the hit points from the LIDAR (only used for drawing):
    private Vector3[] hitPoints;

    // For drawing only:
    private Vector3 lastScanPosition, lastScanForward;

    // List of observations made by the LIDAR, as well as the observation index and if the observation is valid or not:
    private ExtendedObservation[] observations;

    // The landmark candidates are the valid observations returned from Douglas Peucker algorithm,
    // that corresponds to convex corners (more stable as concave ones). The rejected candidates are
    // the other points returned from Douglas Peucker algorithm.
    private List<int> landmarkCandidates, rejectedCandidates;

    // Start is called before the first frame update
    void Start() {
        hitPoints = new Vector3[raycastCount];
        observations = new ExtendedObservation[raycastCount];
    }

    // Update is called once per frame. We do nothing here: the raycasting computation is done
    // when calling PerformLidarScan(). This solves some synchronization issues:
    void Update() {}

    public void OnDrawGizmos() {
        if (drawCorners && landmarkCandidates != null) {
            Gizmos.color = Color.green;

            foreach (int index in landmarkCandidates)
                Gizmos.DrawSphere(hitPoints[index], 0.1f);
        }

        if (drawCorners && rejectedCandidates != null) {
            Gizmos.color = Color.red;

            foreach (int index in rejectedCandidates)
                Gizmos.DrawSphere(hitPoints[index], 0.1f);
        }

        if (drawRays && observations != null) {
            Vector3 direction = lastScanForward;

            foreach (ExtendedObservation observation in observations) {
                Gizmos.color = observation.isValid ? Color.red : Color.white;
                Gizmos.DrawRay(lastScanPosition, direction * observation.r);

                direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            }
        }
    }

    // Use raycasting to compute the observations made by the LIDAR:
    public void PerformLidarScan() {
        // Save the current position and orientation of the LIDAR, to draw gizmos later:
        lastScanPosition = transform.position;
        lastScanForward = transform.TransformDirection(Vector3.forward);

        // Compute the raycast intersections with the environment, and update the observations:
        ComputeObservations();

        // From the observations, detect the ones that correspond to corners (that are used as landmarks):
        DetectCorners();
    }

    // Use raycasting to compute the observations made by the LIDAR:
    private void ComputeObservations() {
        float observationAngle = 0;
        Vector3 direction = transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < observations.Length; i++) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, raycastDistance)) {
                hitPoints[i] = hit.point;

                // Used for localisation, mapping, etc...
                observations[i] = new ExtendedObservation(hit.distance, observationAngle, i, true);
            }
            else {
                hitPoints[i] = transform.position + direction * raycastDistance;

                // Used for localisation, mapping, etc...
                observations[i] = new ExtendedObservation(raycastDistance, observationAngle, i, false);
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            observationAngle += 2 * Mathf.PI / raycastCount;
        }
    }

    private void DetectCorners() {
        if (observations == null)
            return;

        landmarkCandidates = new List<int>();
        rejectedCandidates = new List<int>();

        // Use Douglas Peucker algorithm to reduce the set of observations made by the LIDAR:
        int[] subset = DouglasPeucker(observations, DouglasPeuckerEpsilon);

        int count = subset.Length;

        // Use the same naming convention as the paper "Corner Detection for Room Mapping of Fire Fighting Robot":
        for(int i = 0; i < count; i++) {
            ExtendedObservation curr = observations[subset[i]];
            if (!curr.isValid) {
                rejectedCandidates.Add(subset[i]);
                continue;
            }

            ExtendedObservation prev = observations[subset[(i + count - 1) % count]];
            ExtendedObservation next = observations[subset[(i + 1) % count]];

            float b = prev.r, c = curr.r, e = next.r;

            // Detect if the current observation is a corner using cosine law:
            float a = Mathf.Sqrt(b * b + c * c - 2 * b * c * Mathf.Cos(curr.theta - prev.theta));
            float d = Mathf.Sqrt(c * c + e * e - 2 * c * e * Mathf.Cos(next.theta - curr.theta));

            float angleB = Mathf.Acos((a*a + c*c - b*b) / (2*a*c));
            float angleE = Mathf.Acos((d*d + c*c - e*e) / (2*d*c));

            // Angle in degrees of the corner:
            float theta = Mathf.Rad2Deg * (angleB + angleE);

            if (theta > 220)
                landmarkCandidates.Add(subset[i]);
            else
                rejectedCandidates.Add(subset[i]);
        }
    }

    private int[] DouglasPeucker(ExtendedObservation[] observations, float epsilon) {
        // Get the position of the observations in the referential of the LIDAR (we don't need the vehicle
        // state estimate to do do):
        (int, Vector2)[] points = new (int, Vector2)[observations.Length];
        foreach (ExtendedObservation observation in observations) {
            float r = observation.r;
            float theta = observation.theta;

            Vector2 position = new Vector2(r * Mathf.Cos(theta), r * Mathf.Sin(theta));
            points[observation.index] = (observation.index, position);
        }

        // Run Douglas Peucker algorithm on the set of points we just built:
        points = DouglasPeucker(points, 0, points.Length - 1, epsilon);
        int startIndex = 0, endIndex = points.Length;

        // Since Douglas Peucker algorithm always keeps the first and last points in the list,
        // we still have to check if these points are necessary:
        if (points.Length > 2) {
            (int, Vector2)
                first = points[0],
                second = points[1],
                beforeLast = points[points.Length - 2],
                last = points[points.Length - 1];

            Vector2 u = (second.Item2 - beforeLast.Item2).normalized;
            Vector2 v1 = first.Item2 - second.Item2;
            Vector2 v1_along_u = Vector2.Dot(v1, u) * u;
            float d1 = (v1 - v1_along_u).magnitude;

            Vector2 v2 = last.Item2 - second.Item2;
            Vector2 v2_along_u = Vector2.Dot(v2, u) * u;
            float d2 = (v2 - v2_along_u).magnitude;

            // Keep the first and last point only if their orthogonal distance from u is big enough:
            if (d1 <= epsilon) startIndex = 1;
            if (d2 <= epsilon) endIndex = points.Length - 1;
        }

        // Convert the list of points into a list of observation indices:
        int[] result = new int[Mathf.Max(0, endIndex - startIndex)];
        for (int i = startIndex; i < endIndex; i++)
            result[i - startIndex] = points[i].Item1;

        return result;
    }

    // Run douglas peucker algorithm on the given list of points, between start and end (inclusives):
    private (int, Vector2)[] DouglasPeucker((int, Vector2)[] points, int start, int end, float epsilon) {
        Vector2 u = (points[end].Item2 - points[start].Item2).normalized;

        // Find the point with maximum orthogonal distance:
        int index = -1;
        float maxDistance = 0;

        for (int i = start + 1; i < end; i++) {
            Vector2 v = points[i].Item2 - points[start].Item2;
            Vector2 v_along_u = Vector2.Dot(v, u) * u;
            float distance = (v - v_along_u).magnitude;

            if (distance >= maxDistance) {
                maxDistance = distance;
                index = i;
            }
        }

        if (maxDistance > epsilon) {
            (int, Vector2)[] list1 = DouglasPeucker(points, start, index, epsilon);
            (int, Vector2)[] list2 = DouglasPeucker(points, index, end, epsilon);

            (int, Vector2)[] result = new (int, Vector2)[list1.Length + list2.Length - 1];
            System.Array.Copy(list1, result, list1.Length - 1);
            System.Array.Copy(list2, 0, result, list1.Length - 1, list2.Length);

            return result;
        }
        else
            return new (int, Vector2)[] { points[start], points[end] };
    }

    public (float, float) GetLocalPosition() {
        return (transform.localPosition.z, -transform.localPosition.x);
    }

    public ExtendedObservation[] GetExtendedObservations() {
        return observations;
    }

    // Use the static landmark candidates to update our state estimate:
    public List<Observation> GetLandmarkCandidates() {
        List<Observation> landmarks = new List<Observation>();
        
        foreach(int index in landmarkCandidates) {
            Observation observation = observations[index].ToObservation();
            if (worldModel.IsStatic(observation))
                landmarks.Add(observation);
        }

        return landmarks;
    }

    public void DrawObservation(Observation observation, Color color) {
        float duration = 0.02f;

        Vector3 direction = transform.TransformDirection(Vector3.forward);
        direction = Quaternion.AngleAxis(-observation.theta * Mathf.Rad2Deg, Vector3.up) * direction;

        Debug.DrawRay(transform.position, direction * observation.r, color, duration);
    }
}
