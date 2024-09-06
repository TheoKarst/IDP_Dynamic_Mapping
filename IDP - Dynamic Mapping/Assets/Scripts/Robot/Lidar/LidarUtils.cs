using System.Collections.Generic;
using UnityEngine;

public class LidarUtils {

    // Run Douglas Peucker algorithm on the given observations, and return the indices of
    // the points returned by the algorithm:
    public static int[] DouglasPeucker(Observation[] observations, float epsilon) {
        // Get the position of the observations, in the referential of the LIDAR. We
        // don't need the state estimate to do so:
        Vector2[] points = new Vector2[observations.Length];
        for (int i = 0; i < observations.Length; i++) {
            float r = observations[i].r;
            float theta = observations[i].theta;

            points[i] = new Vector2(r * Mathf.Cos(theta), r * Mathf.Sin(theta));
        }

        // Run Douglas Peucker algorithm on the set of points, to get a subset of points:
        return DouglasPeuckerIndices(points, epsilon, true);
    }

    // Return the corners with an angle greater than "minAngleDegrees":
    public static List<int> ExtractConvexCorners(Observation[] observations, float minAngleDegrees) {
        List<int> corners = new List<int>();

        int count = observations.Length;

        // Use the same naming convention as the paper "Corner Detection for Room Mapping of Fire Fighting RobotManager":
        for (int i = 0; i < count; i++) {
            Observation curr = observations[i];

            // Observations outOfRange cannot be identified as corners:
            if (curr.outOfRange)
                continue;

            Observation prev = observations[(i + count - 1) % count];
            Observation next = observations[(i + 1) % count];

            float b = prev.r, c = curr.r, e = next.r;

            // Detect if the current observation is a corner using cosine law:
            float a = Mathf.Sqrt(b * b + c * c - 2 * b * c * Mathf.Cos(curr.theta - prev.theta));
            float d = Mathf.Sqrt(c * c + e * e - 2 * c * e * Mathf.Cos(next.theta - curr.theta));

            float angleB = Mathf.Acos((a * a + c * c - b * b) / (2 * a * c));
            float angleE = Mathf.Acos((d * d + c * c - e * e) / (2 * d * c));

            // Angle in degrees of the corner:
            float theta = Mathf.Rad2Deg * (angleB + angleE);

            // Keep only the convex corners:
            if (theta >= minAngleDegrees)
                corners.Add(i);
        }

        return corners;
    }

    // From a subset of observations, return the corners with an angle greater than "minAngleDegrees":
    public static List<int> ExtractConvexCorners(Observation[] observations, int[] indices, float minAngleDegrees) {
        List<int> corners = new List<int>();

        int count = indices.Length;

        // Use the same naming convention as the paper "Corner Detection for Room Mapping of Fire Fighting RobotManager":
        for (int i = 0; i < count; i++) {
            Observation curr = observations[indices[i]];

            // Observations outOfRange cannot be identified as corners:
            if (curr.outOfRange)
                continue;

            Observation prev = observations[indices[(i + count - 1) % count]];
            Observation next = observations[indices[(i + 1) % count]];

            float b = prev.r, c = curr.r, e = next.r;

            // Detect if the current observation is a corner using cosine law:
            float a = Mathf.Sqrt(b * b + c * c - 2 * b * c * Mathf.Cos(curr.theta - prev.theta));
            float d = Mathf.Sqrt(c * c + e * e - 2 * c * e * Mathf.Cos(next.theta - curr.theta));

            float angleB = Mathf.Acos((a * a + c * c - b * b) / (2 * a * c));
            float angleE = Mathf.Acos((d * d + c * c - e * e) / (2 * d * c));

            // Angle in degrees of the corner:
            float theta = Mathf.Rad2Deg * (angleB + angleE);

            // Keep only the convex corners:
            if (theta >= minAngleDegrees)
                corners.Add(indices[i]);
        }

        return corners;
    }

    // Among the list of observations from the LIDAR, return the one that are static, according to the given world model:
    public static List<Observation> GetStaticObservations(Observation[] observations, List<int> subset, WorldModel worldModel, VehicleModel model, VehicleState stateEstimate) {
        List<Observation> landmarks = new List<Observation>();

        worldModel.Cleanup();
        foreach (int index in subset) {
            Observation observation = observations[index];

            // Use the vehicle model and vehicle state to get the world space position of
            // the observation:
            Vector2 worldPosition = model.ComputeObservationPositionEstimate(stateEstimate, observation);

            if (worldModel.IsStatic(worldPosition))
                landmarks.Add(observation);
        }

        // Sort the landmarks by distance, as near landmarks are usually more precise:
        landmarks.Sort((Observation a, Observation b) => {
            if (a.r < b.r) return -1;
            if (a.r > b.r) return 1;
            return 0;
        });

        return landmarks;
    }

    // Run Douglas Peucker algorithm on the given list of points, and return the indices
    // of the points kept by the algorithm. If loop = true, we suppose that the set of 
    // points represent a loop, and we thus perform a further check to see if we need
    // to keep the first and last point of the curve (that are otherwise always kept
    // by Douglas Peucker algorithm):
    public static int[] DouglasPeuckerIndices(Vector2[] points, float epsilon, bool loop) {
        int[] indices = new int[points.Length];
        for (int i = 0; i < indices.Length; i++) indices[i] = i;

        // Perform Douglas Peucker algorithm on the set of points:
        indices = DouglasPeuckerIndices(points, indices, 0, points.Length - 1, epsilon);

        // If loop = true, check if we need to keep the first and last point of the curve:
        if (loop && indices.Length >= 4) {
            Vector2 first = points[indices[0]];
            Vector2 second = points[indices[1]];
            Vector2 beforeLast = points[indices[indices.Length - 2]];
            Vector2 last = points[indices[indices.Length - 1]];

            Vector2 u = Vector2.Perpendicular(second - beforeLast).normalized;
            float d1 = Mathf.Abs(Vector2.Dot(first - second, u));
            float d2 = Mathf.Abs(Vector2.Dot(last - second, u));

            // Keep the first and last point only if their orthogonal distance from u is big enough:
            if (d1 <= epsilon || d2 <= epsilon) {
                int startIndex = d1 <= epsilon ? 1 : 0;
                int endIndex = d2 <= epsilon ? indices.Length - 1 : indices.Length;

                int[] result = new int[endIndex - startIndex];
                System.Array.Copy(indices, startIndex, result, 0, result.Length);
                return result;
            }
        }

        return indices;
    }

    public static int[] DouglasPeuckerIndices(Vector2[] points, int[] indices, int start, int end, float epsilon) {
        Vector2 startPoint = points[indices[start]];
        Vector2 endPoint = points[indices[end]];

        Vector2 u = new Vector2(startPoint.y - endPoint.y, endPoint.x - startPoint.x).normalized;

        // Find the point with maximum orthogonal distance:
        int index = -1;
        float maxDistance = 0;

        for (int i = start + 1; i < end; i++) {
            float distance = Mathf.Abs(Vector2.Dot(points[indices[i]] - startPoint, u));

            if (distance >= maxDistance) {
                maxDistance = distance;
                index = i;
            }
        }

        if (maxDistance > epsilon) {
            int[] indices1 = DouglasPeuckerIndices(points, indices, start, index, epsilon);
            int[] indices2 = DouglasPeuckerIndices(points, indices, index, end, epsilon);

            int[] result = new int[indices1.Length + indices2.Length - 1];
            System.Array.Copy(indices1, result, indices1.Length - 1);
            System.Array.Copy(indices2, 0, result, indices1.Length - 1, indices2.Length);

            return result;
        }
        else
            return new int[] { indices[start], indices[end] };
    }

    public static List<Vector2> ComputeWipeShapePoints(Vector2[] curve, float stepRadius, int minLookAhead, float epsilon) {
        List<Vector2> result = new List<Vector2>();

        if (curve.Length == 0)
            return result;

        int lastPivotIndex = 0;
        Vector2 pivot = curve[0];
        result.Add(pivot);

        // During the algorithm, we are only using stepRadius²:
        float stepRadiusSq = stepRadius * stepRadius;

        int index = 1;
        while (index < curve.Length) {
            int lookahead = 1;
            int lastExitIndex = -1;
            bool previousInsideRadius = true;

            while (index < curve.Length && (previousInsideRadius || lookahead <= minLookAhead)) {
                Vector2 next = curve[index];
                float OB_magSq = (next - pivot).sqrMagnitude;

                if (OB_magSq >= stepRadiusSq) {
                    if (previousInsideRadius) {
                        lastExitIndex = index;

                        if (lookahead >= minLookAhead) break;
                    }

                    previousInsideRadius = false;
                }
                else
                    previousInsideRadius = true;

                lookahead++;
                index++;
            }

            // If lastExitIndex == -1, this means we reached the end of the curve.
            // We thus keep the last point and exit:
            if(lastExitIndex == -1) {
                result.Add(curve[curve.Length - 1]);
                break;
            }

            // Else, we know that the point lastExitIndex-1 was in the circle of
            // center pivot and radius stepRadius, while the point lastExitIndex
            // was outide. There is thus an intersection between this circle and
            // the segment [curve[lastExitIndex-1], curve[lastExitIndex]]. We compute
            // this intersection and use it as the next pivot:
            else {
                index = lastExitIndex;

                Vector2 next = curve[lastExitIndex];
                Vector2 OB = next - pivot;
                float OB_magSq = OB.sqrMagnitude;

                Vector2 BA = curve[lastExitIndex - 1] - next;
                float a = BA.sqrMagnitude;
                float b = Vector2.Dot(OB, BA);
                float c = OB_magSq - stepRadiusSq;

                float t = (-b - Mathf.Sqrt(b * b - a * c)) / a;

                Vector2 nextPivot = next + BA * t;

                // Before adding this new point in the result, we check if we are not missing an important point:
                bool abort = false;

                if (lastPivotIndex + 1 < index) {
                    Vector2 u = Vector2.Perpendicular(nextPivot - pivot).normalized;

                    // Find the point with maximum orthogonal distance:
                    int maxIndex = -1;
                    float maxDistance = 0;

                    for (int i = lastPivotIndex + 1; i < index; i++) {
                        float distance = Vector2.Dot(curve[i]- pivot, u);

                        if (distance > maxDistance) {
                            maxIndex = i;
                            maxDistance = distance;
                        }
                    }

                    // If we found a point between pivot and next pivot with an orthoginal
                    // distance bigger than epsilon, to the segment [pivot, next pivot],
                    // then we have to keep this point. We will thus add it to the result
                    // and use it as the next pivot:
                    if (maxDistance > epsilon) {
                        lastPivotIndex = maxIndex;
                        pivot = curve[maxIndex];
                        result.Add(pivot);
                        index = maxIndex + 1;
                        abort = true;
                    }
                }

                // If we found no important pivot to keep before nextPivot, we use
                // nextPivot as the next pivot:
                if (!abort) {
                    lastPivotIndex = lastExitIndex - 1;
                    pivot = nextPivot;
                    result.Add(pivot);
                }
            }
        }

        return result;
    }

    public static Vector2[] DouglasPeucker(Vector2[] points, float epsilon) {
        return DouglasPeucker(points, 0, points.Length - 1, epsilon);
    }

    private static Vector2[] DouglasPeucker(Vector2[] points, int start, int end, float epsilon) {
        Vector2 startPoint = points[start];
        Vector2 endPoint = points[end];

        Vector2 u = new Vector2(startPoint.y - endPoint.y, endPoint.x - startPoint.x).normalized;

        // Find the point with maximum orthogonal distance:
        int index = -1;
        float maxDistance = 0;

        for (int i = start + 1; i < end; i++) {
            float distance = Mathf.Abs(Vector2.Dot(points[i] - startPoint, u));

            if (distance >= maxDistance) {
                maxDistance = distance;
                index = i;
            }
        }

        if (maxDistance > epsilon) {
            Vector2[] list1 = DouglasPeucker(points, start, index, epsilon);
            Vector2[] list2 = DouglasPeucker(points, index, end, epsilon);

            Vector2[] result = new Vector2[list1.Length + list2.Length - 1];
            System.Array.Copy(list1, result, list1.Length - 1);
            System.Array.Copy(list2, 0, result, list1.Length - 1, list2.Length);

            return result;
        }
        else
            return new Vector2[] { points[start], points[end] };
    }
}