using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

public class WipeShapeUtils {
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
        if(remove == null)
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
}
