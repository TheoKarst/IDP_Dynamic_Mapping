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
    private float CriticalAlphaRadians;

    [Tooltip("Maximum distance between two consecutive points to be matched to the same circle cluster")]
    public float CircleCriticalDistance = 0.2f;

    [Tooltip("Minimum length of a line (smaller lines are considered as circle clusters)")]
    public float LineMinLength = 0.3f;

    [Header("Geometry Matching:")]

    [Tooltip("Maximum angle (in degrees) between two lines to be matched together")]
    public float LineMaxMatchAngle = 10f;
    private float LineMaxMatchAngleRadians;

    [Tooltip("Maximum distance between the endpoint of two lines to be matched together")]
    public float LineMaxEndpointMatchDistance = 0.1f;

    [Tooltip("Maximum distance between two lines to be matched together")]
    public float LineMaxMatchDistance = 0.2f;

    [Tooltip("Maximum distance between two circles to be matched together")]
    public float CircleMaxMatchDistance = 0.2f;

    [Header("Geometry removing")]

    [Tooltip("Extent of the wipe triangle built for new lines")]
    public float WipeTriangleExtent = 0.1f;

    [Tooltip("Minimum distance between a circle and a line in the model")]
    public float MinCircleLineDistance = 0.5f;

    [Header("Drawing")]
    public bool drawPoints = true;
    public bool drawLines = true;
    public bool drawCircles = true;
    public bool drawCurrentLines = true;

    // Match to each observation (rho, theta) from the LIDAR a point in world space coordinate.
    // A point may be null if the corresponding observation was wrong (observation.distance == -1):
    private Point[] currentPoints;

    // Current lines and circles used to represent the environment:
    private List<Line> modelLines = new List<Line>();
    private List<Circle> modelCircles = new List<Circle>();

    // For debugging: List of lines that were built during this frame:
    private List<Line> debugCurrentLines = new List<Line>();

    // TEST:
    private int newLineCount = 0;

    // Start is called before the first frame update
    void Start() {
        CriticalAlphaRadians = Mathf.Deg2Rad * CriticalAlpha;
        LineMaxMatchAngleRadians = Mathf.Deg2Rad * LineMaxMatchAngle;
    }

    // Update is called once per frame
    void Update() {
        // Get the vehicle state estimate from Kalman Filter:
        VehicleState vehicleState; Matrix<double> stateCovariance;
        (vehicleState, stateCovariance) = controller.GetRobotStateEstimate();

        // Get the observations from the LIADR:
        ExtendedObservation[] observations = lidar.GetExtendedObservations();

        // During the initialisation, the LIDAR observations may be null. In this case
        // there is nothing to do:
        if (observations == null)
            return;

        // Get points from the LIDAR:
        currentPoints = ComputePoints(vehicleState, stateCovariance, observations);

        // Then use the points to perform lines and circles extraction:
        List<Line> lines; List<Circle> circles;
        (lines, circles) = ClusterExtraction(currentPoints);

        // For debugging:
        if(drawCurrentLines) {
            debugCurrentLines.Clear();
            foreach(Line line in lines) {
                Line copy = new Line(line);
                copy.lineColor = Color.yellow;
                debugCurrentLines.Add(copy);
            }
        }

        // Use the lines from the current frame to update the model lines:
        Vector2 sensorPosition = controller.GetVehicleModel().GetSensorPosition(vehicleState);
        WipeTriangle[] triangles = UpdateModelLines(sensorPosition, lines);

        // Use the circles from the current frame to update the model circles:
        UpdateModelCircles(circles, triangles);
    }

    public void OnDrawGizmos() {
        if(drawCurrentLines && debugCurrentLines != null) {
            foreach(Line line in debugCurrentLines)
                line.DrawGizmos();
        }

        if (drawPoints && currentPoints != null) {
            foreach (Point point in currentPoints)
                if(point != null)
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
    private Point[] ComputePoints(VehicleState vehicleState, Matrix<double> stateCovariance, ExtendedObservation[] observations) {

        // Convert observations into points, using the vehicle state estimate:
        VehicleModel model = controller.GetVehicleModel();
        
        Point[] points = new Point[observations.Length];
        for(int i = 0; i < observations.Length; i++) {
            ExtendedObservation observation = observations[i];

            // If the observation is valid, estimate its position using the robot state:
            if (observation.isValid) {
                Vector<double> position; Matrix<double> covariance;
                (position, covariance) =
                    model.ComputeObservationPositionEstimate(vehicleState, stateCovariance, observation.ToObservation());

                float x = (float) position[0], y = (float) position[1], theta = observation.theta;
                points[i] = new Point(x, y, theta, covariance);
            }
        }

        return points;
    }

    // Cluster extraction (lines and circles):
    private (List<Line>, List<Circle>) ClusterExtraction(Point[] points) {
        List<Line> extractedLines = new List<Line>();
        List<Circle> extractedCircles = new List<Circle>();

        // We first try to match the first point with a line, then with a circle.
        // If we are building a line, we should have lineBuilder == null.
        // If we are building a circle, we should have circleBuilder == null.
        LineBuilder lineBuilder = null;
        CircleBuilder circleBuilder = null;

        foreach (Point currentPoint in points) {
            if (currentPoint == null) continue;

            // Initialisation: this should be executed just for the first non null point:
            if(lineBuilder == null && circleBuilder == null) {
                lineBuilder = new LineBuilder(currentPoint);
                continue;
            }

            // 1. If we are currently building a line:
            if(lineBuilder != null) {
                Point previousPoint = lineBuilder.GetLastPoint();

                // Try to match the current point with the current line:
                bool condition1 = Point.Dist(previousPoint, currentPoint) <= PointCriticalDistance;
                bool condition2 = lineBuilder.PointsCount() < 3 || lineBuilder.DistanceFrom(currentPoint) <= LineCriticalDistance;
                bool condition3 = Point.AngularDifference(previousPoint, currentPoint) <= CriticalAlphaRadians;
                
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

    private WipeTriangle[] UpdateModelLines(Vector2 sensorPosition, List<Line> currentLines) {

        // List of wipe triangles built from the current lines:
        WipeTriangle[] wipeTriangles = new WipeTriangle[currentLines.Count];

        // Drawing: Reset the color of the model lines to red:
        foreach (Line line in modelLines) line.lineColor = Color.red;

        // Try to match the current lines with the lines in the model:
        for (int i = 0; i < currentLines.Count; i++) {
            Line line = currentLines[i];

            // Build the wipe triangle for this line:
            wipeTriangles[i] = line.BuildWipeTriangle(sensorPosition, WipeTriangleExtent);

            // The best matching line we found in the model, as well as the
            // distance between this line and the one in the model:
            Line bestMatch = null;
            float minDistance = -1;

            string logMsg = "Line " + i + ": [" + line.LogParams() + "]\n";
            foreach (Line matchCandidate in modelLines) {
                logMsg += line.LogMatchCandidate(matchCandidate, LineMaxMatchAngleRadians, LineMaxEndpointMatchDistance);
                logMsg += "\t[" + matchCandidate.LogParams() + "=>" 
                    + Utils.ScientificNotation(line.ComputeNormDistance(matchCandidate)) + "]\t";
                logMsg += "[" + Utils.ScientificNotation(line.ComputeCenterDistance(matchCandidate)) + "]";

                // First test: compare the angle difference and endpoints distance between the two lines:
                if (line.IsMatchCandidate(matchCandidate, LineMaxMatchAngleRadians, LineMaxEndpointMatchDistance)) {

                    // Second test: Compute the Mahalanobis distance between the lines:
                    if (line.ComputeNormDistance(matchCandidate) < 5) {

                        // Last step: Find the nearest line among the remaining candidates:
                        // TODO: Check that this implementation matches the paper description:
                        float centerDistance = line.ComputeCenterDistance(matchCandidate);

                        if (bestMatch == null || centerDistance < minDistance) {
                            bestMatch = matchCandidate;
                            minDistance = centerDistance;
                            logMsg += ": Possible match !";
                        }
                    }
                }
                logMsg += "\n";
            }

            // If a match is found and near enough, use this line to update the match state estimate:
            if (bestMatch != null && minDistance <= LineMaxMatchDistance) {
                bestMatch.lineColor = Color.blue;       // Blue color for matched lines
                bestMatch.UpdateLineUsingMatching(line);

                // Update all the model (except the matching line) using the wipe triangle of this line:
                modelLines = wipeTriangles[i].UpdateLines(modelLines, bestMatch, LineMinLength);
                modelLines.Add(bestMatch);
                logMsg += "=> Match found !";
            }

            // Else, just add this new line to the model:
            else {
                line.lineColor = Color.green;       // Green color for new lines
                modelLines = wipeTriangles[i].UpdateLines(modelLines, null, LineMinLength);
                modelLines.Add(line);
                logMsg += "=> New line !";
                Debug.Log("New line (" + (++newLineCount) + ") !");
            }

            // Debug.Log(logMsg);
        }

        // List of wipe triangles that we built, that will be used to remove inconsistent circles:
        return wipeTriangles;
    }

    public void UpdateModelCircles(List<Circle> currentCircles, WipeTriangle[] wipeTriangles) {
        List<Circle> newCircles = new List<Circle>();

        // Drawing: Reset the color of the model circles to red:
        foreach (Circle circle in modelCircles) circle.circleColor = Color.red;

        // Try to match the current circles with the circles in the model:
        foreach (Circle circle in currentCircles) {
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
            if (bestMatch != null && minDistance <= CircleMaxMatchDistance) {
                bestMatch.UpdateCircleUsingMatching(circle);
                bestMatch.circleColor = Color.blue;
            }

            // Else, just add the circle to the model:
            else {
                newCircles.Add(circle);
                circle.circleColor = Color.green;
            }
        }

        // Add the new lines in the model:
        foreach (Circle circle in newCircles)
            modelCircles.Add(circle);

        // Use the wipe triangles of the new lines to delete inconsistent circles:
        foreach(WipeTriangle triangle in wipeTriangles) 
            modelCircles = triangle.UpdateCircles(modelCircles);
    }
}
