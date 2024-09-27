using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

public class Filtering {
    /// <summary>
    /// Computes the local position of the observations made by a LIDAR. We don't need the vehicle
    /// state to do so, and this can be used to apply Douglas Peucker algorithm, since the result of
    /// the algorithm in the global frame or the local frame of the LIDAR will be the same 
    /// </summary>
    private static Vector2[] ComputeLocalPosition(Observation[] observations) {
        Vector2[] positions = new Vector2[observations.Length];

        for (int i = 0; i < observations.Length; i++) {
            float r = observations[i].r;
            float theta = observations[i].theta;

            positions[i] = new Vector2(r * Mathf.Cos(theta), r * Mathf.Sin(theta));
        }

        return positions;
    }

    /// <summary>
    /// For each observation of a LIDAR, remove all the observations in the "alpha-cone" of that observation
    /// </summary>
    /// <param name="points">List of positions of the observations</param>
    /// <param name="angles">List of angles of the observations from the LIDAR in radians</param>
    /// <param name="alpha">Angle of the "alpha-cone" in radians</param>
    /// <returns>The indices of the observations to remove after filtering</returns>
    public static bool[] AlphaFilter(Vector2[] points, float[] angles, float alpha) {
        Assert.IsTrue(points.Length == angles.Length);

        // Register each point that should be deleted:
        bool[] remove = new bool[points.Length];

        // For each point, remove all the points in the "alpha-cone" of this point:
        for (int i = 0; i < points.Length; i++) {
            if (remove[i])
                continue;

            // 1. Remove points in the cone in counter-clockwise order, between theta and theta + alpha:
            int index = i;
            float theta = angles[i];
            float stopAngle = theta + alpha;

            // cos(x + PI/2) = -sin(x); sin(x + PI/2) = cos(x)
            Vector2 u = new Vector2(-Mathf.Sin(stopAngle), Mathf.Cos(stopAngle));

            while (theta < stopAngle) {
                index++;

                // If we reached the last point, return to the first one:
                if (index >= points.Length) {
                    index = 0;
                    stopAngle -= 2 * Mathf.PI;
                }

                theta = angles[index];
                float dotU = Vector2.Dot(points[index] - points[i], u);

                if (dotU <= 0)
                    remove[index] = true;
                else
                    break;
            }

            // 2. Remove points in the cone in clockwise order, between theta and theta - alpha:
            index = i;
            theta = angles[i];
            stopAngle = theta - alpha;

            // cos(x - PI/2) = sin(x); sin(x - PI/2) = -cos(x)
            u = new Vector2(Mathf.Sin(stopAngle), -Mathf.Cos(stopAngle));

            while (theta > stopAngle) {
                index--;

                // If we reached the first point, return to the last one:
                if (index <= 0) {
                    index = points.Length - 1;
                    stopAngle += 2 * Mathf.PI;
                }

                theta = angles[index];
                float dotU = Vector2.Dot(points[index] - points[i], u);

                if (dotU <= 0)
                    remove[index] = true;
                else
                    break;
            }
        }

        return remove;
    }

    /// <summary>
    /// Runs Douglas-Peucker algorithm on the given positions between start (inclusive)
    /// and end (inclusive)
    /// </summary>
    /// <param name="positions">Positions of the points of the shape to filter</param>
    /// <param name="epsilon">Parameter of the algorithm that defines how much the curve will be filtered</param>
    /// <param name="remove">Mask of positions to remove (by default, no position is removed)</param>
    /// <returns>The updated mask of positions to remove</returns>
    public static bool[] DouglasPeucker(Vector2[] positions, float epsilon, bool[] remove = null) {

        // If there are less than two points, we always keep them:
        if (positions.Length <= 2)
            return new bool[] { true, true };

        Stack<(int, int)> stack = new Stack<(int, int)>();
        stack.Push((0, positions.Length - 1));

        // Mask of positions to remove:
        if (remove == null)
            remove = new bool[positions.Length];

        while (stack.Count != 0) {
            (int start, int end) = stack.Pop();
            Vector2 startPoint = positions[start];
            Vector2 endPoint = positions[end];

            if (end - start <= 1)
                continue;

            // Create a unit vector orthogonal to the line between start_point and end_point:
            Vector2 u = Vector2.Perpendicular(endPoint - startPoint).normalized;

            // For each point between start (exclusive) and end (exclusive), compute the
            // orthogonal distance between that point and the line [start_point, end_point]:
            int maxIndex = -1;
            float maxDistance = 0;

            for (int i = start + 1; i < end; i++) {
                if (remove[i])
                    continue;

                float distance = Mathf.Abs(Vector2.Dot(positions[i] - startPoint, u));

                if (maxIndex == -1 || distance > maxDistance) {
                    maxIndex = i;
                    maxDistance = distance;
                }
            }

            if (maxDistance > epsilon) {
                stack.Push((start, maxIndex));
                stack.Push((maxIndex, end));
            }
            else {
                for (int i = start + 1; i < end; i++)
                    remove[i] = true;
            }
        }

        return remove;
    }

    /// <summary>
    /// Runs Douglas-Peucker algorithm on the given observations
    /// </summary>
    /// <param name="observations">Observations made by a LIDAR</param>
    /// <param name="epsilon">Parameter of the algorithm that defines how much the curve will be filtered</param>
    /// <returns>The filtered observations</returns>
    public static Observation[] DouglasPeucker(Observation[] observations, float epsilon) {
        // Compute the local position of the observations:
        Vector2[] positions = ComputeLocalPosition(observations);

        // Compute which observations to remove:
        bool[] remove = DouglasPeucker(positions, epsilon);

        // Return the observations to keep:
        int count = 0;
        for(int i = 0; i < remove.Length; i++)
            if (!remove[i])
                count++;

        Observation[] filtered = new Observation[count];
        for(int i = remove.Length - 1; i >= 0; i--) {
            if (!remove[i])
                filtered[--count] = observations[i];
        }

        return filtered;
    }

    // Return the corners with an angle greater than "minAngleDegrees":
    /// <summary>
    /// Among the given observations, return the one corresponding to concave corners
    /// </summary>
    /// <param name="observations">The observations to filter</param>
    /// <param name="minAngleDegrees">Minimum angle in degrees of the observations (to extract concave corners,
    /// this angle should be between 180 and 360°)</param>
    /// <returns>The filtered observations</returns>
    public static List<Observation> ExtractConcaveCorners(Observation[] observations, float minAngleDegrees) {
        List<Observation> corners = new List<Observation>();

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
                corners.Add(curr);
        }

        return corners;
    }

    /// <summary>
    /// Among the list of observations from the LIDAR, return the one that are static, according to the given world model
    /// </summary>
    /// <param name="observations">Observations made by a LIDAR</param>
    /// <param name="worldModel">Model of the environment, used to decide which landmarks are static or dynamic</param>
    /// <param name="model">Model of the vehicle, used to compute the position of the observations</param>
    /// <param name="stateEstimate">State estimate of the vehicle, used to compute the position of the observations</param>
    /// <returns></returns>
    public static List<Observation> GetStaticObservations(List<Observation> observations, GridMapBresenham worldModel, VehicleModel model, VehicleState stateEstimate) {
        List<Observation> landmarks = new List<Observation>();

        foreach (Observation observation in observations) {

            // Use the vehicle model and vehicle state to get the world space position of
            // the observation:
            Vector2 worldPosition = model.ComputeObservationPositionEstimate(stateEstimate, observation);

            // Use the world model to check if the observation corresponds to a static object or not:
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
}