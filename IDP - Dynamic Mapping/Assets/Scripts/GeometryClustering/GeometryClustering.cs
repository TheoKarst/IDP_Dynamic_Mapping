using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;

public class GeometryClustering : MonoBehaviour
{
    public Lidar lidar;
    public RobotController controller;

    [Header("Geometry Extraction:")]

    [Tooltip("Maximum distance between two consecutive points to be matched to the same line")]
    public float PointCriticalDistance = 0.2f;

    [Tooltip("Maximum orthogonal distance of a point to a matching line")]
    public float LineCriticalDistance = 0.06f;

    [Tooltip("Maximum angle (in degrees) between two consecutive points to be matched to the same line")]
    public float CriticalAlpha = 1f;

    [Tooltip("Maximum distance between two consecutive points to be matched to the same circle cluster")]
    public float CircleCriticalDistance = 0.2f;

    [Tooltip("Minimum length of a line (smaller lines are considered as circle clusters)")]
    public float LineMinLength = 0.3f;

    [Header("Geometry Matching:")]

    [Tooltip("Maximum angle between two lines to be matched together")]
    public float LineMaxMatchAngle = 10f;

    [Tooltip("Maximum distance between the endpoint of two lines to be matched together")]
    public float LineMaxEndpointMatchDistance = 0.1f;

    [Tooltip("Maximum distance between two lines to be matched together")]
    public float LineMaxMatchDistance = 0.2f;

    [Tooltip("Maximum distance between two circles to be matched together")]
    public float CircleMaxMatchDistance = 0.2f;

    [Tooltip("Extent of the wipe triangle built for new lines")]
    public float WipeTriangleExtent = 0.1f;

    [Header("Drawing")]
    public bool drawPoints = true;
    public bool drawLines = true;
    public bool drawCircles = true;

    // Points currently extracted from this frame (used for drawing):
    private List<Point> currentPoints;

    // Current lines and circles used to represent the environment:
    private List<Line> modelLines = new List<Line>();
    private List<Circle> modelCircles = new List<Circle>();

    // Start is called before the first frame update
    void Start() {
        
    }

    // Update is called once per frame
    void Update() {
        // Get the vehicle state estimate from Kalman Filter:
        VehicleState vehicleState; Matrix<float> stateCovariance;
        (vehicleState, stateCovariance) = controller.GetRobotStateEstimate();

        // Get the observations from the LIADR:
        Observation[] observations = lidar.GetObservations();

        // During the initialisation, the LIDAR observations may be null. In this case
        // there is nothing to do:
        if (observations == null)
            return;

        // Get points from the LIDAR:
        currentPoints = ComputePoints(vehicleState, stateCovariance, observations);

        // Then use the points to perform lines and circles extraction:
        List<Line> lines; List<Circle> circles;
        (lines, circles) = ClusterExtraction(currentPoints);

        // Use the lines from the current frame to update the model lines:
        Vector2 sensorPosition = controller.GetVehicleModel().GetSensorPosition(vehicleState);
        UpdateModelLines(sensorPosition, lines);

        // Use the circles from the current frame to update the model circles:
        UpdateModelCircles(circles);

        Debug.Log("Points: " + currentPoints.Count + 
            "; Lines: " + modelLines.Count + 
            "; Circles: " + modelCircles.Count);
    }

    public void OnDrawGizmos() {
        if (drawPoints && currentPoints != null) {
            foreach (Point point in currentPoints)
                point.DrawGizmos();
        }

        if (drawLines && modelLines != null) {
            foreach (Line line in modelLines)
                line.DrawGizmos();
        }

        if(drawCircles && modelCircles != null) {
            foreach (Circle circle in modelCircles)
                circle.DrawGizmos();
        }
    }

    // Use the LIDAR observations and the vehicle state estimate from the Kalman Filter
    // to compute the estimated position of all the observations of the LIDAR in world space:
    private List<Point> ComputePoints(VehicleState vehicleState, Matrix<float> stateCovariance, Observation[] observations) {

        // Convert observations into points, using the vehicle state estimate:
        VehicleModel model = controller.GetVehicleModel();
        
        List<Point> points = new List<Point>();
        foreach (Observation observation in observations) {

            // If the observation is valid, estimate its position using the robot state:
            if (observation.r >= 0) {
                Vector<float> position; Matrix<float> covariance;
                (position, covariance) =
                    model.ComputeObservationPositionEstimate(vehicleState, stateCovariance, observation);

                float x = position[0], y = position[1], theta = observation.theta;
                points.Add(new Point(x, y, theta, covariance));
            }
        }

        return points;
    }

    // Cluster extraction (lines and circles):
    private (List<Line>, List<Circle>) ClusterExtraction(List<Point> points) {
        List<Line> extractedLines = new List<Line>();
        List<Circle> extractedCircles = new List<Circle>();

        if(points.Count == 0)
            return (extractedLines, extractedCircles);

        // We first try to match the first point with a line, then with a circle.
        // If we are building a line, we should have lineBuilder == null.
        // If we are building a circle, we should have circleBuilder == null.
        LineBuilder lineBuilder = new LineBuilder(points[0]);
        CircleBuilder circleBuilder = null;

        for(int i = 1; i < points.Count; i++) {
            Point currentPoint = points[i];

            // 1. If we are currently building a line:
            if(lineBuilder != null) {
                Point previousPoint = lineBuilder.GetLastPoint();

                // Try to match the current point with the current line:
                bool condition1 = Point.Dist(previousPoint, currentPoint) <= PointCriticalDistance;
                bool condition2 = lineBuilder.PointsCount() < 3 || lineBuilder.DistanceFrom(currentPoint) <= LineCriticalDistance;
                bool condition3 = Point.AngularDifference(previousPoint, currentPoint) <= Mathf.Deg2Rad * CriticalAlpha;
                
                // If the three conditions are met, we can add the point to the line:
                if (condition1 && condition2 && condition3) {
                    lineBuilder.AddPoint(currentPoint);
                    continue;
                }

                // Else, if the current line is long enough to be extracted, then extract it, and add the current
                // point in a new line:
                else if (lineBuilder.PointsCount() >= 3 && lineBuilder.Length() >= LineMinLength) {
                    extractedLines.Add(lineBuilder.Build());
                    lineBuilder = new LineBuilder(currentPoint);
                    continue;
                }

                // Otherwise, convert the current line into a circle cluster, and continue to process this circle:
                else {
                    circleBuilder = lineBuilder.ToCircle();
                    lineBuilder = null;
                }
            }

            // 2. If we are currently building a circle:

            // If the current point can be added to the current circle, add it:
            if (circleBuilder.DistanceFrom(currentPoint) <= CircleCriticalDistance) {
                circleBuilder.AddPoint(currentPoint);
            }

            // Otherwise, extract the current circle and add the current point in a new line:
            else {
                // A circle cluster should at least contain 2 points:
                if (circleBuilder.PointsCount() >= 2) {
                    extractedCircles.Add(circleBuilder.Build());
                }

                circleBuilder = null;
                lineBuilder = new LineBuilder(currentPoint);
            }
        }

        // Finally, extract the current line or current circle if necessary:
        if (lineBuilder != null && lineBuilder.PointsCount() >= 3) {
            extractedLines.Add(lineBuilder.Build());
        }
        else if(circleBuilder != null && circleBuilder.PointsCount() >= 2) {
            extractedCircles.Add(circleBuilder.Build());
        }

        return (extractedLines, extractedCircles);
    }

    private void UpdateModelLines(Vector2 sensorPosition, List<Line> currentLines) {
        // All the lines that were added to the model, plus the lines from the model that were updated:
        List<Line> newLines = new List<Line>();

        // All the other lines of the model:
        List<Line> oldLines = new List<Line>();

        // List of booleans saying for each line of the model if it was matched with a line of the
        // current frame or not:
        bool[] matched = new bool[modelLines.Count];

        // Try to match the current lines with the lines in the model:
        foreach (Line line in currentLines) {
            // The best matching line we found in the model, as well as the
            // distance between this line and the one in the model:
            int bestMatch = -1;
            float minDistance = -1;

            for(int i = 0; i < modelLines.Count; i++) {
                Line matchCandidate = modelLines[i];

                // First test: compare the angle difference and endpoints distance between the two lines:
                if (line.IsMatchCandidate(matchCandidate, LineMaxMatchAngle, LineMaxEndpointMatchDistance)) {
                    
                    // Second test: Compute the Mahalanobis distance between the lines:
                    if(line.ComputeNormDistance(matchCandidate) < 5) {

                        // Last step: Find the nearest line among the remaining candidates:
                        // TODO: Check that this implementation matches the paper description:
                        float centerDistance = line.ComputeCenterDistance(matchCandidate);

                        if(bestMatch == -1 || centerDistance < minDistance) {
                            bestMatch = i;
                            minDistance = centerDistance;
                        }
                    }
                }
            }

            // If a match is found and near enough, use this line to update the match state estimate:
            if (bestMatch != -1 && minDistance <= LineMaxMatchDistance) {
                Line match = modelLines[bestMatch];
                match.UpdateLineUsingMatching(line);

                if (!matched[bestMatch]) {
                    newLines.Add(match);
                    matched[bestMatch] = true;
                }
            }

            // Else, just add the line to the model:
            else
                newLines.Add(line);
        }

        // Get from the model unmatched lines:
        for(int i = 0; i < modelLines.Count; i++)
            if (!matched[i])
                oldLines.Add(modelLines[i]);

        // Use the new lines to update the old ones:
        foreach(Line line in newLines) {
            WipeTriangle triangle = line.BuildWipeTriangle(sensorPosition, WipeTriangleExtent);
            oldLines = triangle.UpdateLines(oldLines, LineMinLength);
        }

        // Add the updated old lines and the new lines to the model:
        modelLines.Clear();
        foreach (Line line in oldLines) { line.lineColor = Color.red; modelLines.Add(line); }
        foreach (Line line in newLines) { line.lineColor = Color.green; modelLines.Add(line); }
    }

    public void UpdateModelCircles(List<Circle> currentCircles) {
        List<Circle> newCircles = new List<Circle>();

        // Try to match the current circles with the circles in the model:
        foreach(Circle circle in currentCircles) {
            Circle bestMatch = null;
            float minDistance = -1;

            foreach(Circle matchCandidate in modelCircles) {
                float distance = circle.DistanceFrom(matchCandidate);

                if(bestMatch == null || distance < minDistance) {
                    bestMatch = matchCandidate;
                    minDistance = distance;
                }
            }

            // If a match is found, use this circle to update the match state estimate:
            if (bestMatch != null && minDistance <= CircleMaxMatchDistance)
                bestMatch.UpdateCircleUsingMatching(circle);
            
            // Else, just add the circle to the model:
            else
                newCircles.Add(circle);
        }

        // Add the new lines in the model:
        foreach (Circle circle in newCircles)
            modelCircles.Add(circle);
    }
}
