using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

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

    [Tooltip("Margin angle (in degrees) of the wipe triangles")]
    public float WipeTriangleInsideAngleMargin = 1f;
    private float WipeTriangleInsideAngleMarginRadians;

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
    private LinkedList<Line> modelLines = new LinkedList<Line>();
    private List<Circle> modelCircles = new List<Circle>();

    // For debugging: List of lines that were built during this frame:
    private List<Line> debugCurrentLines = new List<Line>();

    // TEST:
    private int newLineCount = 0;

    // Start is called before the first frame update
    void Start() {
        CriticalAlphaRadians = Mathf.Deg2Rad * CriticalAlpha;
        LineMaxMatchAngleRadians = Mathf.Deg2Rad * LineMaxMatchAngle;
        WipeTriangleInsideAngleMarginRadians = Mathf.Deg2Rad * WipeTriangleInsideAngleMargin;

        // Allocate the list of points once, since it's since is not going to change:
        currentPoints = new Point[lidar.raycastCount];
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
        Profiler.BeginSample("Compute Points");
        ComputePoints(vehicleState, stateCovariance, observations);
        Profiler.EndSample();
        
        // Then use the points to perform lines and circles extraction:
        List<Line> lines; List<Circle> circles;

        Profiler.BeginSample("Cluster Extraction");
        (lines, circles) = ClusterExtraction(currentPoints);
        Profiler.EndSample();

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

        Profiler.BeginSample("Update Model Lines");
        List<WipeTriangle> triangles = UpdateModelLines(sensorPosition, lines);
        Profiler.EndSample();

        // Use the circles from the current frame to update the model circles:
        Profiler.BeginSample("Update Model Circles");
        UpdateModelCircles(circles, triangles);
        Profiler.EndSample();

        Debug.Log("Points: " + currentPoints.Length + "; Lines: " + modelLines.Count 
            + "; Circles: " + modelCircles.Count + "; Triangles: " + triangles.Count);
    }

    public void OnDrawGizmos() {
        if(drawCurrentLines && debugCurrentLines != null) {
            foreach(Line line in debugCurrentLines)
                line.DrawGizmos();
        }

        if (drawPoints && currentPoints != null) {
            foreach (Point point in currentPoints)
                if(point.isValid)
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
    private void ComputePoints(VehicleState vehicleState, Matrix<double> stateCovariance, ExtendedObservation[] observations) {

        // Convert observations into points, using the vehicle state estimate:
        VehicleModel model = controller.GetVehicleModel();
        
        for(int i = 0; i < observations.Length; i++) {
            ExtendedObservation observation = observations[i];

            // If the observation is valid, estimate its position using the robot state:
            if (observation.isValid) {
                Vector<double> position; Matrix<double> covariance;
                (position, covariance) =
                    model.ComputeObservationPositionEstimate(vehicleState, stateCovariance, observation.ToObservation());

                float x = (float)position[0], y = (float)position[1], theta = observation.theta;
                currentPoints[i] = new Point(x, y, theta, covariance, true);
            }
            else
                currentPoints[i] = new Point(0, 0, 0, null, false);
        }
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
            if (!currentPoint.isValid) continue;

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
        if (lineBuilder != null && lineBuilder.PointsCount() >= 3 && lineBuilder.Length() >= LineMinLength) {
            extractedLines.Add(lineBuilder.Build());
        }
        else if(circleBuilder != null && circleBuilder.PointsCount() >= 2) {
            extractedCircles.Add(circleBuilder.Build());
        }

        return (extractedLines, extractedCircles);
    }

    private List<WipeTriangle> UpdateModelLines(Vector2 sensorPosition, List<Line> currentLines) {

        // List of wipe triangles built from the current lines:
        List<WipeTriangle> wipeTriangles = new List<WipeTriangle>();

        // Drawing: Reset the color of the model lines to red:
        foreach (Line line in modelLines) line.lineColor = Color.red;

        // Try to match the current lines with the lines in the model:
        for (int i = 0; i < currentLines.Count; i++) {
            Line line = currentLines[i];

            // Build the wipe triangle for this line:
            WipeTriangle wipeTriangle = line.BuildWipeTriangle(sensorPosition, WipeTriangleExtent, WipeTriangleInsideAngleMarginRadians);
            if(wipeTriangle != null)
                wipeTriangles.Add(wipeTriangle);

            // The best matching line we found in the model, as well as the
            // distance between this line and the one in the model:
            Line bestMatch = null;
            float minDistance = -1;

            Profiler.BeginSample("Lines matching");
            foreach (Line matchCandidate in modelLines) {
                // First test: compare the angle difference and endpoints distance between the two lines:
                if (line.IsMatchCandidate(matchCandidate, LineMaxMatchAngleRadians, LineMaxEndpointMatchDistance)) {

                    // Second test: Compute the Mahalanobis distance between the lines:
                    if (line.ComputeNormDistance(matchCandidate) < 5) {

                        // Last step: Find the nearest line among the remaining candidates:
                        // TODO: Check that this implementation matches the paper description:
                        Profiler.BeginSample("Compute Center Distance");
                        float centerDistance = line.ComputeCenterDistance(matchCandidate);
                        Profiler.EndSample();

                        if (bestMatch == null || centerDistance < minDistance) {
                            bestMatch = matchCandidate;
                            minDistance = centerDistance;
                        }
                    }
                }
            }
            Profiler.EndSample();

            // If a match is found and near enough, use this line to update the match state estimate:
            Profiler.BeginSample("Line Update");
            if (bestMatch != null && minDistance <= LineMaxMatchDistance) {
                bestMatch.lineColor = Color.blue;       // Blue color for matched lines
                bestMatch.UpdateLineUsingMatching(line);

                // Update all the model (except the matching line) using the wipe triangle of this line:
                if (wipeTriangle != null)
                    wipeTriangle.UpdateLines(modelLines, bestMatch, LineMinLength);

                // modelLines.Add(bestMatch);
            }

            // Else, just add this new line to the model:
            else {
                line.lineColor = Color.green;       // Green color for new lines

                if (wipeTriangle != null)
                    wipeTriangle.UpdateLines(modelLines, null, LineMinLength);

                modelLines.AddLast(line);
                //Debug.Log("New line (" + (++newLineCount) + ") !");
            }
            Profiler.EndSample();
        }

        // List of wipe triangles that we built, that will be used to remove inconsistent circles:
        return wipeTriangles;
    }

    // Update the lines using a wipe shape instead of wipe triangles:
    private void UpdateModelLines(Vector2 sensorPosition, List<Line> currentLines, WipeShape wipeShape) {
        // Drawing: Reset the color of the model lines to red:
        foreach (Line line in modelLines) line.lineColor = Color.red;

        // Update the lines in the model using the wipe shape:


        // Try to match the current lines with the lines in the model:
        for (int i = 0; i < currentLines.Count; i++) {
            Line line = currentLines[i];

            // The best matching line we found in the model, as well as the
            // distance between this line and the one in the model:
            Line bestMatch = null;
            float minDistance = -1;

            Profiler.BeginSample("Lines matching");
            foreach (Line matchCandidate in modelLines) {
                // First test: compare the angle difference and endpoints distance between the two lines:
                if (line.IsMatchCandidate(matchCandidate, LineMaxMatchAngleRadians, LineMaxEndpointMatchDistance)) {

                    // Second test: Compute the Mahalanobis distance between the lines:
                    if (line.ComputeNormDistance(matchCandidate) < 5) {

                        // Last step: Find the nearest line among the remaining candidates:
                        // TODO: Check that this implementation matches the paper description:
                        Profiler.BeginSample("Compute Center Distance");
                        float centerDistance = line.ComputeCenterDistance(matchCandidate);
                        Profiler.EndSample();

                        if (bestMatch == null || centerDistance < minDistance) {
                            bestMatch = matchCandidate;
                            minDistance = centerDistance;
                        }
                    }
                }
            }
            Profiler.EndSample();

            // If a match is found and near enough, use this line to update the match state estimate:
            Profiler.BeginSample("Line Update");
            if (bestMatch != null && minDistance <= LineMaxMatchDistance) {
                bestMatch.lineColor = Color.blue;       // Blue color for matched lines
                bestMatch.UpdateLineUsingMatching(line);
            }

            // Else, just add this new line to the model:
            else {
                line.lineColor = Color.green;       // Green color for new lines
                modelLines.AddLast(line);
                //Debug.Log("New line (" + (++newLineCount) + ") !");
            }
            Profiler.EndSample();
        }
    }

    public void UpdateModelCircles(List<Circle> currentCircles, List<WipeTriangle> wipeTriangles) {
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
