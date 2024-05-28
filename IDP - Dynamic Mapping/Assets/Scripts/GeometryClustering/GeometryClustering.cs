using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;

public class GeometryClustering : MonoBehaviour
{
    public Lidar lidar;
    public RobotController controller;

    public float POINT_CRITICAL_DISTANCE = 0.2f;
    public float LINE_CRITICAL_DISTANCE = 0.06f;
    public float CRITICAL_ALPHA = 5;

    public bool drawPoints = true;
    public bool drawLines = true;

    // Points and lines currently extracted from this frame:
    private List<Point> points;
    private List<Line> lines;

    // Start is called before the first frame update
    void Start() {
        
    }

    // Update is called once per frame
    void Update() {
        // Get the observations from the LIADR:
        Observation[] observations = lidar.GetObservations();

        if (observations == null)
            return;

        // Get the vehicle state estimate from Kalman Filter:
        VehicleState vehicleState; Matrix<float> stateCovariance;
        (vehicleState, stateCovariance) = controller.GetRobotStateEstimate();

        // Convert observations into points, using the vehicle state estimate:
        VehicleModel model = controller.GetVehicleModel();
        this.points = new List<Point>();

        foreach(Observation observation in observations) {

            // If the observation is valid, estimate its position using the robot state:
            if(observation.r >= 0) {
                Vector<float> position; Matrix<float> covariance;
                (position, covariance) =
                    model.ComputeObservationPositionEstimate(vehicleState, stateCovariance, observation);

                float x = position[0], y = position[1], theta = observation.theta;
                points.Add(new Point(x, y, theta, covariance));
            }
        }

        // Now use the points to perform line extraction:
        this.lines = LineExtraction(points);
        Debug.Log("Lines count: " + lines.Count);
    }

    public void OnDrawGizmos() {
        if (drawPoints && points != null) {
            foreach (Point point in points)
                point.DrawGizmos();
        }

        if (drawLines && lines != null) {
            foreach (Line line in lines)
                line.DrawGizmos();
        }
    }

    // Line extraction, following section 4.3.1:
    private List<Line> LineExtraction(List<Point> points) {
        List<Line> extractedLines = new List<Line>();
        Line currentLine = new Line();

        foreach (Point point in points) {

            // Get the last point that was added to the line:
            Point lastPoint = currentLine.GetLastPoint();

            if (lastPoint == null) {
                currentLine.AddPoint(point);
            }
            else {
                bool condition1 = Point.Dist(lastPoint, point) <= POINT_CRITICAL_DISTANCE;
                bool condition2 = currentLine.PointsCount() < 3 || currentLine.DistanceFrom(point) <= LINE_CRITICAL_DISTANCE;
                bool condition3 = Point.AngularDifference(lastPoint, point) <= Mathf.Deg2Rad * CRITICAL_ALPHA;

                // If the three conditions are met, we can add the point to the line. Otherwise, the extraction is aborted
                // and the parameters of the line are calculated:
                if (condition1 && condition2 && condition3) {
                    currentLine.AddPoint(point);
                }
                else {
                    if (currentLine.PointsCount() >= 3) {
                        currentLine.Build();
                        extractedLines.Add(currentLine);
                    }

                    currentLine = new Line();
                    currentLine.AddPoint(point);
                }
            }
        }

        if (currentLine.PointsCount() >= 3) {
            currentLine.Build();
            extractedLines.Add(currentLine);
        }

        return extractedLines;
    }
}
