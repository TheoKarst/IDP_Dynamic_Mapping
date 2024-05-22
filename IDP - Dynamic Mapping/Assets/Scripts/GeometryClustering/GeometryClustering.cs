using System.Collections.Generic;
using UnityEngine;

public class GeometryClustering : MonoBehaviour
{
    public Lidar lidar;

    public float POINT_CRITICAL_DISTANCE = 20;
    public float LINE_CRITICAL_DISTANCE = 4;
    public float CRITICAL_ALPHA = 10;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // Line extraction, following section 4.3.1:
    private List<Line> lineExtraction(Point[] points) {
        List<Line> extractedLines = new List<Line>();
        Line currentLine = new Line();

        foreach (Point point in points) {

            // Get the last point that was added to the line:
            Point lastPoint = currentLine.GetLastPoint();

            if (lastPoint == null) {
                currentLine.AddPoint(point);
            }
            else {
                bool condition1 = Point.dist(lastPoint, point) <= POINT_CRITICAL_DISTANCE;
                bool condition2 = currentLine.PointsCount() < 3 || currentLine.DistanceFrom(point) <= LINE_CRITICAL_DISTANCE;
                bool condition3 = Point.angularDifference(lastPoint, point) <= Mathf.Deg2Rad * CRITICAL_ALPHA;

                // If the three conditions are met, we can add the point to the line. Otherwise, the extraction is aborted
                // and the parameters of the line are calculated:
                if (condition1 && condition2 && condition3) {
                    currentLine.AddPoint(point);
                }
                else {
                    if (currentLine.PointsCount() >= 3) {
                        currentLine.ComputeEndpoints();
                        extractedLines.Add(currentLine);
                    }

                    currentLine = new Line();
                    currentLine.AddPoint(point);
                }
            }
        }

        if (currentLine.PointsCount() >= 3) {
            currentLine.ComputeEndpoints();
            extractedLines.Add(currentLine);
        }

        return extractedLines;
    }
}
