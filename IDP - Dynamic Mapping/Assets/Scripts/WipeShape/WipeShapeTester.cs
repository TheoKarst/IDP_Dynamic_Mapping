using MathNet.Numerics.Distributions;
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class WipeShapeTester : MonoBehaviour {

    /*
    public enum FilterMode {
        DELTA_RADIUS,
        MAX_AREA,
        MAX_ANGLE
    }*/

    public Robot robot;
    public GameObject lidarObject;
    public int lidarIndex = 0;

    public bool showInitialPoints = false;
    public bool showSubsampledPoints = false;

    [Space(10)]
    public bool subsample = false;
    public bool douglasPeucker = false;

    [Range(0, 90)]
    public float alphaAngle = 0;
    [Min(0)]
    public float epsilon = 0.01f;

    private Vector2[] positions;
    private Vector2[] wipeShape;
    private int[] subsampledPoints;

    private AugmentedObservation[] lastObservations;

    void Start() {

    }

    // Update is called once per frame
    void Update() {
        if (lastObservations != null && Input.GetKey(KeyCode.S))
            SaveData(lastObservations, "lidar_data.csv");

        // Update the robot state here (since it's done by Unity, we don't need to do so):
        // ...

        // If a new frame of data is available, use it to update the maps:
        if (robot.IsNewFrameAvailable()) {
            // Get the current data of the robot:
            DataloaderRobot.RobotData data = robot.GetCurrentFrame();

            // This script only uses the data from a single LIDAR:
            VehicleModel model = robot.GetVehicleModel();
            AugmentedObservation[] observations = data.observations[lidarIndex];
            lastObservations = observations;

            // Get the world space position of the observations:
            Vector2[] positions = new Vector2[observations.Length];
            for(int i = 0; i < observations.Length; i++) {
                Observation observation = new Observation(observations[i].r, observations[i].theta, lidarIndex);
                positions[i] = model.ComputeObservationPositionEstimate(data.vehicleState, observation);
            }
            this.positions = positions;

            if (subsample) {
                int[] subset = WipeShapeUtils.AlphaFilter(observations, alphaAngle * Mathf.Deg2Rad);
                
                if (douglasPeucker)
                    subset = LidarUtils.DouglasPeuckerIndices(positions, subset, 0, subset.Length - 1, epsilon);

                positions = new Vector2[subset.Length];
                for (int i = 0; i < subset.Length; i++) {
                    positions[i] = this.positions[subset[i]];
                }

                subsampledPoints = subset;
            }

            // Update the shape:
            wipeShape = positions;
        }
    }

    public void OnDrawGizmos() {
        const float height = 0.2f;

        if (positions != null && positions.Length > 0) {
            Gizmos.color = Color.white;
            Vector2 prev = positions[positions.Length - 1];
            foreach (Vector2 point in positions) {
                Gizmos.DrawLine(Utils.To3D(prev, height), Utils.To3D(point, height));
                prev = point;

                if (showInitialPoints)
                    Gizmos.DrawSphere(Utils.To3D(point, height), 0.1f);
            }
        }

        if (wipeShape != null && wipeShape.Length > 0) {
            Gizmos.color = Color.green;
            Vector2 prev = wipeShape[wipeShape.Length - 1];
            foreach (Vector2 point in wipeShape) {
                Gizmos.DrawLine(Utils.To3D(prev, height), Utils.To3D(point, height));
                prev = point;
            }
        }

        if(showSubsampledPoints && subsampledPoints != null) {
            Gizmos.color = Color.green;
            foreach(int index in subsampledPoints) {
                Gizmos.DrawSphere(Utils.To3D(positions[index], height), 0.1f);
            }
        }
    }

    /*
    private static List<int> MinSubsample(AugmentedObservation[] observations, float deltaAngle) {
        List<int> result = new List<int>();

        AugmentedObservation lastObservation = null;
        int lastObservationIndex = -1;

        int index = 0;
        while (index < observations.Length) {
            // Find the observation with the minimum radius in the list of observations
            // having an angle between first.theta and first.theta + deltaAngle:
            AugmentedObservation first = observations[index];

            int minIndex = index;
            AugmentedObservation minObservation = first;

            index++;
            while(index < observations.Length) {
                AugmentedObservation current = observations[index];

                if (current.theta - first.theta > deltaAngle)
                    break;

                if (current.r < minObservation.r) {
                    minIndex = index;
                    minObservation = current;
                }

                index++;
            }

            // Before adding minIndex to the list, we have to check that the angle
            // between lastObservation and minObservation is greater than deltaAngle.
            // If this is not the case, we keep among them the one with the smaller 
            // radius:
            if(lastObservation != null && minObservation.theta - lastObservation.theta < deltaAngle) {
                if(minObservation.r < lastObservation.r) {
                    lastObservation = minObservation;
                    lastObservationIndex = minIndex;
                }
            }
            else {
                if (lastObservation != null)
                    result.Add(lastObservationIndex);

                lastObservation = minObservation;
                lastObservationIndex = minIndex;
            }
        }

        if (lastObservation != null)
            result.Add(lastObservationIndex);
        
        return result;
    }

    private List<int> HighPassSubsample(AugmentedObservation[] observations, float deltaAngle) {
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
            float maxFilterValue = MaxFilter(observations, index);
            AugmentedObservation maxObservation = first;

            index++;
            while (index < observations.Length) {
                AugmentedObservation current = observations[index];

                if (current.theta - first.theta > deltaAngle)
                    break;

                float filterValue = MaxFilter(observations, index);
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
    
    private float MaxFilter(AugmentedObservation[] observations, int index) {
        if(filterMode == FilterMode.DELTA_RADIUS)
            return MaxDeltaFilter(observations, index);

        if(filterMode == FilterMode.MAX_ANGLE)
            return MaxAngleFilter(observations, index);

        if(filterMode == FilterMode.MAX_AREA)
            return MaxAreaFilter(observations, index);

        return 0;
    }

    private float MaxDeltaFilter(AugmentedObservation[] observations, int index) {
        int last = observations.Length - 1;

        AugmentedObservation prev = observations[index == 0 ? last : index - 1];
        AugmentedObservation curr = observations[index];
        AugmentedObservation next = observations[index == last ? 0 : index + 1];

        return prev.r + next.r - 2 * curr.r;
    }

    private float MaxAreaFilter(AugmentedObservation[] observations, int index) {
        int last = observations.Length - 1;

        Observation prev = observations[index == 0 ? last : index - 1].ToObservation();
        Observation curr = observations[index].ToObservation();
        Observation next = observations[index == last ? 0 : index + 1].ToObservation();

        DataloaderRobot.RobotData data = robot.GetCurrentFrame();
        VehicleModel model = robot.GetVehicleModel();
        Vector2 A = model.ComputeObservationPositionEstimate(data.vehicleState, prev);
        Vector2 B = model.ComputeObservationPositionEstimate(data.vehicleState, curr);
        Vector2 C = model.ComputeObservationPositionEstimate(data.vehicleState, next);

        Vector2 BA = A - B;
        Vector2 BC = C - B;

        // Return the cross product between BA and BC:
        return BA.x * BC.y - BA.y * BC.x;
    }

    private float MaxAngleFilter(AugmentedObservation[] observations, int index) {
        int last = observations.Length - 1;

        AugmentedObservation prev = observations[index == 0 ? last : index - 1];
        AugmentedObservation curr = observations[index];
        AugmentedObservation next = observations[index == last ? 0 : index + 1];


        float b = prev.r, c = curr.r, e = next.r;

        // Detect if the current observation is a corner using cosine law:
        float a = Mathf.Sqrt(b * b + c * c - 2 * b * c * Mathf.Cos(curr.theta - prev.theta));
        float d = Mathf.Sqrt(c * c + e * e - 2 * c * e * Mathf.Cos(next.theta - curr.theta));

        float angleB = Mathf.Acos((a * a + c * c - b * b) / (2 * a * c));
        float angleE = Mathf.Acos((d * d + c * c - e * e) / (2 * d * c));

        // Angle in radians of the corner:
        return angleB + angleE;
    }

    public List<int> HighPassFilter(AugmentedObservation[] observations, float minValue) {
        List<int> result = new List<int>();

        int last = observations.Length - 1;

        float filtered;
        float prev = observations[last].r;
        for(int i = 0; i < last; i++) {
            filtered = prev + observations[i+1].r - 2 * observations[i].r;
            prev = observations[i].r;

            if(filtered >= minValue)
                result.Add(i);
        }

        // Check if we keep the last observation:
        filtered = prev + observations[0].r - 2 * observations[last].r;
        if (filtered >= minValue)
            result.Add(last);

        return result;
    }

    private static List<int> ComputeInterestPoints(AugmentedObservation[] observations, List<int> corners, float minAngleBetweenPoints) {
        List<int> result = new List<int>();

        for(int i = 1; i < corners.Count - 1; i++) {
            AugmentedObservation prev = observations[corners[i - 1]];
            AugmentedObservation current = observations[corners[i]];
            AugmentedObservation next = observations[corners[i + 1]];

            // If both observations are too near to each other, we keep only the one with
            // the shortest radius:
            if(next.theta - current.theta < minAngleBetweenPoints) {
                if (next.r < current.r)
                    continue;
            }

            if(current.theta - prev.theta > 2 * minAngleBetweenPoints) {
                // Keep the current observation, and the one "minAngleBetweenPoints" before it:
                int index = corners[i];
                while (index >= 0 && current.theta - observations[index].theta < minAngleBetweenPoints)
                    index--;

                if (index >= 0)
                    result.Add(index);
            }

            result.Add(corners[i]);
        }

        return result;
    }
    */

    private void SaveData(AugmentedObservation[] observations, string filename) {
        StreamWriter file = File.CreateText(filename);

        file.WriteLine("range;angle");

        foreach (AugmentedObservation observation in observations)
            file.WriteLine(observation.r + ";" + observation.theta);

        file.Flush();
        file.Close();

        Debug.Log("Lidar data was successfully saved in the file: " + filename);
    }
}