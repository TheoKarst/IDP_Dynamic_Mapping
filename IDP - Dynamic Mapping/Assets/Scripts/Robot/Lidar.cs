using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;

public class Lidar {

    // GameObject representing the LIDAR:
    private GameObject lidar;

    // Number of raycasts produced by the LIDAR:
    private int raycastCount;

    // Maximum distance of the raycasts:
    private float raycastDistance;

    // Epsilon we are using in Douglas Peucker algorithm to simplify the contour shape of the LIDAR,
    // before corners extraction:
    private float douglasPeuckerEpsilon;

    // List of observations made by the LIDAR, as well as the observation index
    // and if the observation is valid (a hit was found) or not:
    private ExtendedObservation[] observations;

    // List of observations that we get after running Douglas Peucker algorithm on the list of observations:
    private List<int> filteredCorners;

    // Subset of filteredCorners, that correspond to convex corners (used as landmarks candidates):
    private List<int> filteredConvexCorners;

    // Used for drawing only:
    private Vector3[] hitPoints;                        // Real position of the hit points
    private Vector3 lastScanPosition, lastScanForward;  // Real position of the LIDAR during last scan

    public Lidar(GameObject lidar, RobotManager.LidarParams lidarParams) {
        this.lidar = lidar;
        this.raycastCount = lidarParams.raycastCount;
        this.raycastDistance = lidarParams.raycastDistance;
        this.douglasPeuckerEpsilon = lidarParams.douglasPeuckerEpsilon;

        this.hitPoints = new Vector3[raycastCount];
        this.observations = new ExtendedObservation[raycastCount];
    }

    public void DrawGizmos(bool drawRays, bool drawCorners) {
        if (drawCorners && filteredCorners != null) {
            Gizmos.color = Color.red;

            foreach (int index in filteredCorners)
                Gizmos.DrawSphere(hitPoints[index], 0.2f);
        }

        if (drawCorners && filteredConvexCorners != null) {
            Gizmos.color = Color.green;

            foreach (int index in filteredConvexCorners)
                Gizmos.DrawSphere(hitPoints[index], 0.2f);
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

    public void Update() {
        // Save the current position and orientation of the LIDAR, to draw gizmos later:
        lastScanPosition = lidar.transform.position;
        lastScanForward = lidar.transform.TransformDirection(Vector3.forward);

        // Use raycasting to compute the observations from the LIDAR:
        ComputeObservations();

        // Detect the corners among the previous observations:
        DetectCorners();
    }

    // Use raycasting to compute the observations made by the LIDAR:
    private void ComputeObservations() {

        // Use raycasting to compute the current observations:
        float observationAngle = 0;
        Vector3 direction = lidar.transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < observations.Length; i++) {
            RaycastHit hit;
            if (Physics.Raycast(lidar.transform.position, direction, out hit, raycastDistance)) {
                hitPoints[i] = hit.point;

                // Used for localisation, mapping, etc...
                observations[i] = new ExtendedObservation(hit.distance, observationAngle, i, true);
            }
            else {
                hitPoints[i] = lidar.transform.position + direction * raycastDistance;

                // Used for localisation, mapping, etc...
                observations[i] = new ExtendedObservation(raycastDistance, observationAngle, i, false);
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
            observationAngle += 2 * Mathf.PI / raycastCount;
        }
    }

    private void DetectCorners() {
        filteredConvexCorners = new List<int>();
        filteredCorners = new List<int>();

        // Use Douglas Peucker algorithm to reduce the set of observations made by the LIDAR:
        int[] subset = DouglasPeucker(observations, douglasPeuckerEpsilon);

        int count = subset.Length;

        // Use the same naming convention as the paper "Corner Detection for Room Mapping of Fire Fighting RobotManager":
        for(int i = 0; i < count; i++) {
            ExtendedObservation curr = observations[subset[i]];
            filteredCorners.Add(subset[i]);

            if (!curr.isValid)
                continue;

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
                filteredConvexCorners.Add(subset[i]);
        }
    }

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

    // Position of the LIDAR on the robot:
    public (float, float) GetLocalPosition() {
        return (lidar.transform.localPosition.z, -lidar.transform.localPosition.x);
    }

    public ExtendedObservation[] GetExtendedObservations() {
        return observations;
    }

    // Use the static landmark candidates to update our state estimate:
    public List<Observation> GetLandmarkCandidates(WorldModel worldModel) {
        List<Observation> landmarks = new List<Observation>();
        
        foreach(int index in filteredConvexCorners) {
            Observation observation = observations[index].ToObservation();
            if (worldModel.IsStatic(observation))
                landmarks.Add(observation);
        }

        // Sort the landmarks by distance, as near landmarks are usually more precise:
        landmarks.Sort((Observation a, Observation b) => { 
            if(a.r <  b.r) return -1;
            if(a.r > b.r) return 1;
            return 0;
        });

        return landmarks;
    }

    public void DrawObservation(Observation observation, Color color) {
        float duration = 0.02f;

        Vector3 direction = lidar.transform.TransformDirection(Vector3.forward);
        direction = Quaternion.AngleAxis(-observation.theta * Mathf.Rad2Deg, Vector3.up) * direction;

        Debug.DrawRay(lidar.transform.position, direction * observation.r, color, duration);
    }
}
