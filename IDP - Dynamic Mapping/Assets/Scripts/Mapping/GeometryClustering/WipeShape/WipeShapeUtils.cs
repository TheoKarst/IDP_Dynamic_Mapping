using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

public class WipeShapeUtils {
    public static List<int> HighPassSubsample(AugmentedObservation[] observations, float deltaAngle) {
        List<int> result = new List<int>();

        AugmentedObservation lastObservation = null;
        int lastObservationIndex = -1;
        float lastObservationFilterValue = -1;

        int index = 0;
        while (index < observations.Length) {
            // Find the observation with the minimum radius in the list of observations
            // having an angle between first.theta and first.theta + deltaAngle:
            AugmentedObservation first = observations[index];

            int maxIndex = index;
            float maxFilterValue = MaxAreaFilter(observations, index);
            AugmentedObservation maxObservation = first;

            index++;
            while (index < observations.Length) {
                AugmentedObservation current = observations[index];

                if (current.theta - first.theta > deltaAngle)
                    break;

                float filterValue = MaxAreaFilter(observations, index);
                if (filterValue > maxFilterValue) {
                    maxIndex = index;
                    maxObservation = current;
                    maxFilterValue = filterValue;
                }

                index++;
            }

            // Before adding maxIndex to the list, we have to check that the angle
            // between lastObservation and maxObservation is greater than deltaAngle.
            // If this is not the case, we keep among them the one with the biggest
            // filter value:
            if (lastObservation != null && maxObservation.theta - lastObservation.theta < deltaAngle) {
                if (maxFilterValue > lastObservationFilterValue) {
                    lastObservation = maxObservation;
                    lastObservationIndex = maxIndex;
                    lastObservationFilterValue = maxFilterValue;
                }
            }
            else {
                if (lastObservation != null)
                    result.Add(lastObservationIndex);

                lastObservation = maxObservation;
                lastObservationIndex = maxIndex;
                lastObservationFilterValue = maxFilterValue;
            }
        }

        if (lastObservation != null)
            result.Add(lastObservationIndex);

        return result;
    }

    private static float MaxAreaFilter(AugmentedObservation[] observations, int index) {
        int last = observations.Length - 1;

        Vector2 A = LocalPosition(observations[index == 0 ? last : index - 1]);
        Vector2 B = LocalPosition(observations[index]);
        Vector2 C = LocalPosition(observations[index == last ? 0 : index + 1]);

        Vector2 BA = A - B;
        Vector2 BC = C - B;

        // Return the cross product between BA and BC:
        return BA.x * BC.y - BA.y * BC.x;
    }

    private static Vector2 LocalPosition(AugmentedObservation observation) {
        return new Vector2(
            observation.r * Mathf.Cos(observation.theta),
            observation.r * Mathf.Sin(observation.theta)
        );
    }

    public static int[] AlphaFilter(AugmentedObservation[] observations, float alpha, float clamp) {
        // For each observation, if we have to remove it:
        bool[] remove = new bool[observations.Length];

        // First compute the local position of each observation:
        Vector2[] positions = new Vector2[observations.Length];
        for(int i = 0; i < observations.Length; i++) {
            float r = Mathf.Min(observations[i].r, clamp);
            float theta = observations[i].theta;

            positions[i] = new Vector2(r * Mathf.Cos(theta), r * Mathf.Sin(theta));
        }

        // Then for each observation, remove all the observations in the "alpha-cone"
        // of of this observation:
        for (int i = 0; i < observations.Length; i++) {
            if (remove[i])
                continue;

            // 1. Remove observations in the cone in counter-clockwise order, between theta and theta + alpha:
            int index = i;
            float theta = observations[i].theta;
            float stopAngle = theta + alpha;

            // cos(x + PI/2) = -sin(x); sin(x + PI/2) = cos(x)
            Vector2 u = new Vector2(-Mathf.Sin(stopAngle), Mathf.Cos(stopAngle));

            while (theta < stopAngle) {
                index++;

                // If we reached the last observation, return to the first one:
                if (index >= observations.Length) {
                    index = 0;
                    stopAngle -= 2 * Mathf.PI;
                }

                theta = observations[index].theta;
                float dotU = Vector2.Dot(positions[index] - positions[i], u);

                if (dotU <= 0)
                    remove[index] = true;
                else
                    break;
            }

            // 2. Remove observations in the cone in clockwise order, between theta and theta - alpha:
            index = i;
            theta = observations[i].theta;
            stopAngle = theta - alpha;

            // cos(x - PI/2) = sin(x); sin(x - PI/2) = -cos(x)
            u = new Vector2(Mathf.Sin(stopAngle), -Mathf.Cos(stopAngle));

            while (theta > stopAngle) {
                index--;

                // If we reached the first observation, return to the last one:
                if (index <= 0) {
                    index = observations.Length - 1;
                    stopAngle += 2 * Mathf.PI;
                }

                theta = observations[index].theta;
                float dotU = Vector2.Dot(positions[index] - positions[i], u);

                if (dotU <= 0)
                    remove[index] = true;
                else
                    break;
            }
        }

        // Count the number of observations we still have:
        int count = 0;
        for (int i = 0; i < remove.Length; i++)
            if (!remove[i])
                count++;

        // Return the indices of the remaining observations:
        int[] result = new int[count];

        int resultIndex = 0;
        for (int i = 0; i < remove.Length; i++) {
            if (!remove[i]) {
                result[resultIndex] = i;
                resultIndex++;
            }
        }

        return result;
    }

    public static int[] AlphaFilter(Vector2[] points, float[] angles, float alpha) {
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

        // Count the number of observations we still have:
        int count = 0;
        for (int i = 0; i < remove.Length; i++)
            if (!remove[i])
                count++;

        // Return the indices of the remaining points:
        int[] result = new int[count];

        int resultIndex = 0;
        for (int i = 0; i < remove.Length; i++) {
            if (!remove[i]) {
                result[resultIndex] = i;
                resultIndex++;
            }
        }

        return result;
    }
}
