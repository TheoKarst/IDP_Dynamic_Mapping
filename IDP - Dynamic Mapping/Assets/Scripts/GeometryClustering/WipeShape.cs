using System.Collections.Generic;
using UnityEngine;

public class WipeShape {
    private Vector2 center;
    private List<Point> points = new List<Point>();

    public WipeShape(Vector2 center, List<Point> points) {
        this.center = center;
        this.points = points;

        /*
        string log = "";
        for (int i = 1; i < points.Count; i++) {
            log += Mathf.Rad2Deg * points[i].angle + "; ";
            if (points[i - 1].angle >= points[i].angle)
                Debug.LogError("Invalid points for the wipe shape !");
        }
        Debug.Log("Wipe shape angles: [" + log + "]");*/
    }

    public void DrawGizmos(float height) {
        if (points.Count == 0)
            return;

        Point prev = points[points.Count - 1];
        foreach (Point point in points) {
            Gizmos.DrawLine(Utils.To3D(prev.position, height), Utils.To3D(point.position, height));
            prev = point;
        }
    }

    // For each line that was given, find the zones of the lines that are valid or invalid.
    // Only the validity zones of the line are updated, not the line shape.
    // These zones will be used later to update the lines, and remove the invalid parts
    public void UpdateLines(List<Line> modelLines) {
        foreach(Line line in modelLines)
            UpdateLineValidity(line);
    }

    // For each circle that was given, the circle is considered as valid only if its
    // center is outside the WipeShape:
    public void UpdateCircles(List<Circle> modelCircles) {
        foreach(Circle circle in modelCircles)
            UpdateCircleValidity(circle);
    }

    private void UpdateLineValidity(Line line) {
        if (points.Count < 3) {
            Debug.LogError("Wipe Shape has less than 3 points !");
            return;
        }

        // All the indices are modulo n:
        int n = points.Count;

        int startPoint, endPoint, direction, count;

        // Compute if we have to check the intersections between the edges of the shape and the line in clockwise or anticlockwise order:
        if (line.IsLeftOfLine(center)) {
            direction = 1;
            startPoint = FindSection(line.beginPoint) - 1;
            endPoint = FindSection(line.endPoint);
            if (startPoint < 0) startPoint += n;
            count = endPoint - startPoint;
        }
        else {
            direction = -1;
            startPoint = FindSection(line.beginPoint);
            endPoint = FindSection(line.endPoint) - 1;
            if (endPoint < 0) endPoint += n;
            count = startPoint - endPoint;
        }

        // We have startPoint in [0; n[ and endPoint in [0; n[. Thus count is in ]-n; n[. We just have to keep count in [0; n[:
        if (count < 0) count += n;

        int index = startPoint;
        Point current = points[index];
        bool isLeft = line.IsLeftOfLine(current.position);

        // If the start point of the line is outside the shape:
        bool startPointOutside = true;

        List<float> intersections = new List<float>();
        for (int i = 0; i < count; i++) {
            index += direction;
            if (index < 0) index += n;
            else if (index >= n) index -= n;
            Point next = points[index];
            bool nextIsLeft = line.IsLeftOfLine(next.position);

            // Compute startPointOutside:
            if (i == 0) {
                // Rotate (next - current) to point outside the shape:
                Vector2 normal = direction * Vector2.Perpendicular(current.position - next.position);

                // Compute the dot product between n and (line.beginPoint - current). If the dot product is greater than 0, the point is outside the shape:
                startPointOutside = Vector2.Dot(line.beginPoint - current.position, normal) >= 0;

                // if(!startPointOutside) {
                //     Debug.Log("Point: " + line.beginPoint + " is in the shape (Section: "+ startPoint + ") . Normal of (" + current.position + "; " + next.position + "): " + normal + "; Line direction: " + direction);
                // }
            }

            // There may be an intersection only if current and next are on different sides of the line:
            if (nextIsLeft != isLeft)  {
                // Compute the distance between the intersection and the begin point of the line.
                // The returned distance should be between 0 (intersection == beginPoint)
                // and 1 (intersection == endPoint):
                float distance = line.IntersectDistance(current.position, next.position);
                if (distance >= 0 && distance <= 1)
                    intersections.Add(distance);
            }

            current = next;
            isLeft = nextIsLeft;
        }

        line.UpdateValidity(intersections, startPointOutside);
    }

    private void UpdateCircleValidity(Circle circle) {
        int nextIndex = FindSection(circle.position);
        int previousIndex = nextIndex > 0 ? nextIndex - 1 : points.Count - 1;

        Vector2 nextPos = points[nextIndex].position;
        Vector2 prevPos = points[previousIndex].position;

        // Rotate (next - prev) to point outside the shape:
        Vector2 normal = Vector2.Perpendicular(prevPos - nextPos);

        // Compute the dot product between n and (circle.position - prevPos).
        // If the dot product is greater than 0, the point is outside the shape:
        bool valid = Vector2.Dot(circle.position - prevPos, normal) >= 0;

        circle.UpdateValidity(valid);
    }

    // Return the index of a point from the shape, with an angle strictly greater than the given point:
    private int FindSection(Vector2 point) {
        float pointAngle = Mathf.Atan2(point.y - center.y, point.x - center.x);

        // while(pointAngle < points[0].angle) pointAngle += 2*PI
        // if (pointAngle < 0) pointAngle += 2 * Mathf.PI;

        // TEST:
        pointAngle = points[0].angle + Mathf.Repeat(pointAngle - points[0].angle, 2 * Mathf.PI);

        int section = 0;
        while (section < points.Count && points[section].angle < pointAngle)
            section++;

        // Keep the result between 0 and points.Count:
        return section == points.Count ? 0 : section;
    }
}