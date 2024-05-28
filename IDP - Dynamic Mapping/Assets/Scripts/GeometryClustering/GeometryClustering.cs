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

    public bool drawPoints = true;
    public bool drawLines = true;
    public bool drawCircles = true;

    // Points currently extracted from this frame (used for drawing):
    private List<Point> currentPoints;

    // Current lines and circles used to represent the environment:
    private List<LineBuilder> modelLines = new List<LineBuilder>();
    private List<Circle> modelCircles = new List<Circle>();

    // Start is called before the first frame update
    void Start() {
        
    }

    // Update is called once per frame
    void Update() {
        // Get points from the LIDAR:
        currentPoints = ComputePoints();

        // Then use the points to perform lines and circles extraction:
        List<LineBuilder> lines; List<Circle> circles;
        (lines, circles) = ClusterExtraction(currentPoints);
        
        // Try to match the current lines with the lines in the model:
        foreach(LineBuilder line in lines) {
            // The best matching line we found in the model, as well as the
            // Mahalanobis distance between this line and the one in the model:
            LineBuilder bestMatch = null;
            float minDistance = -1;

            foreach(LineBuilder matchCandidate in modelLines) {
                if(line.IsMatchCandidate(matchCandidate, LineMaxMatchAngle, LineMaxEndpointMatchDistance)) {
                    float distance = line.ComputeNormDistance(matchCandidate);

                    if(bestMatch == null || distance < minDistance) {
                        bestMatch = matchCandidate;
                        minDistance = distance;
                    }
                }
            }


        }
    }

    public void OnDrawGizmos() {
        if (drawPoints && currentPoints != null) {
            foreach (Point point in currentPoints)
                point.DrawGizmos();
        }

        if (drawLines && modelLines != null) {
            foreach (LineBuilder line in modelLines)
                line.DrawGizmos();
        }

        if(drawCircles && modelCircles != null) {
            foreach (Circle circle in modelCircles)
                circle.DrawGizmos();
        }
    }

    // Use the LIDAR observations and the vehicle state estimate from the Kalman Filter
    // to compute the estimated position of all the observations of the LIDAR in world space:
    private List<Point> ComputePoints() {

        // Get the observations from the LIADR:
        Observation[] observations = lidar.GetObservations();

        // During the initialisation, the LIDAR observations may be null. In this case
        // there is nothing to do:
        if (observations == null)
            return null;

        // Get the vehicle state estimate from Kalman Filter:
        VehicleState vehicleState; Matrix<float> stateCovariance;
        (vehicleState, stateCovariance) = controller.GetRobotStateEstimate();

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
    private (List<LineBuilder>, List<Circle>) ClusterExtraction(List<Point> points) {
        List<LineBuilder> extractedLines = new List<LineBuilder>();
        List<Circle> extractedCircles = new List<Circle>();

        if(points.Count == 0)
            return (extractedLines, extractedCircles);

        // We first try to match the first point with a line, then with a circle.
        // If we are building a line, we should have currentCircle == null.
        // If we are building a circle, we should have currentLine == null.
        LineBuilder currentLine = new LineBuilder(points[0]);
        Circle currentCircle = null;

        for(int i = 1; i < points.Count; i++) {
            Point currentPoint = points[i];

            // 1. If we are currently building a line:
            if(currentLine != null) {
                Point previousPoint = currentLine.GetLastPoint();

                // Try to match the current point with the current line:
                bool condition1 = Point.Dist(previousPoint, currentPoint) <= PointCriticalDistance;
                bool condition2 = currentLine.PointsCount() < 3 || currentLine.DistanceFrom(currentPoint) <= LineCriticalDistance;
                bool condition3 = Point.AngularDifference(previousPoint, currentPoint) <= Mathf.Deg2Rad * CriticalAlpha;
                
                // If the three conditions are met, we can add the point to the line:
                if (condition1 && condition2 && condition3) {
                    currentLine.AddPoint(currentPoint);
                    continue;
                }

                // Else, if the current line is long enough to be extracted, then extract it, and add the current
                // point in a new line:
                else if (currentLine.PointsCount() >= 3 && currentLine.Length() >= LineMinLength) {
                    currentLine.Build();
                    extractedLines.Add(currentLine);
                    currentLine = new LineBuilder(currentPoint);
                    continue;
                }

                // Otherwise, convert the current line into a circle cluster, and continue to process this circle:
                else {
                    currentCircle = currentLine.ToCircle();
                    currentLine = null;
                }
            }

            // 2. If we are currently building a circle:

            // If the current point can be added to the current circle, add it:
            if (currentCircle.DistanceFrom(currentPoint) <= CircleCriticalDistance) {
                currentCircle.AddPoint(currentPoint);
            }

            // Otherwise, extract the current circle and add the current point in a new line:
            else {
                // A circle cluster should at least contain 2 points:
                if (currentCircle.PointsCount() >= 2) {
                    currentCircle.Build();
                    extractedCircles.Add(currentCircle);
                }

                currentCircle = null;
                currentLine = new LineBuilder(currentPoint);
            }
        }

        // Finally, extract the current line or current circle if necessary:
        if (currentLine != null && currentLine.PointsCount() >= 3) {
            currentLine.Build();
            extractedLines.Add(currentLine);
        }
        else if(currentCircle != null && currentCircle.PointsCount() >= 2) {
            currentCircle.Build();
            extractedCircles.Add(currentCircle);
        }

        return (extractedLines, extractedCircles);
    }
}
