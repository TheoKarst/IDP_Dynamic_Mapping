using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

public class WipeShape {
    private Vector2 center;
    private Vector2[] points;
    private float[] angles;

    public WipeShape(Vector2 center, Vector2[] points) {
        this.center = center;
        this.points = points;

        this.angles = new float[points.Length];
        angles[0] = Mathf.Atan2(points[0].y - center.y,
                                      points[0].x - center.x);
        string logMsg = "[" + angles[0] * Mathf.Rad2Deg;

        for (int i = 1; i < points.Length; i++) {
            float pointAngle = Mathf.Atan2(points[i].y - center.y, 
                                           points[i].x - center.x);

            float step = Mathf.Repeat(pointAngle - angles[i-1], 2 * Mathf.PI);
            if(step >= Mathf.PI)
                Debug.LogError("Assertion error while building the wipe shape");

            angles[i] = angles[i-1] + step;
            logMsg += "; " + angles[i] * Mathf.Rad2Deg;
        }
        if(angles[angles.Length - 1] - angles[0] > 2 * Mathf.PI)
            Debug.LogError("Angles: " + logMsg + "]; First point: " + points[0] + ", Last: " + points[points.Length-1]);

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
        if (points.Length == 0)
            return;

        Gizmos.color = Color.white;
        Vector2 prev = points[points.Length - 1];
        foreach (Vector2 point in points) {
            Gizmos.DrawLine(Utils.To3D(prev, height), Utils.To3D(point, height));
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
        if (points.Length < 3) {
            Debug.LogError("Wipe Shape has less than 3 points !");
            return;
        }

        // All the indices are modulo n:
        int n = points.Length;

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
        Vector2 current = points[index];
        bool isLeft = line.IsLeftOfLine(current);

        // If the start point of the line is outside the shape:
        bool startPointOutside = true;

        List<float> intersections = new List<float>();
        for (int i = 0; i < count; i++) {
            index += direction;
            if (index < 0) index += n;
            else if (index >= n) index -= n;
            Vector2 next = points[index];
            bool nextIsLeft = line.IsLeftOfLine(next);

            // Compute startPointOutside:
            if (i == 0) {
                // Rotate (next - current) to point outside the shape:
                Vector2 normal = direction * Vector2.Perpendicular(current - next);

                // Compute the dot product between n and (line.beginPoint - current). If the dot product is greater than 0, the point is outside the shape:
                startPointOutside = Vector2.Dot(line.beginPoint - current, normal) >= 0;

                // if(!startPointOutside) {
                //    Debug.Log("Point: " + line.beginPoint + " is in the shape (Section: " + startPoint + ") . Normal of (" + current + "; " + next + "): " + normal + "; Line direction: " + direction);
                // }
            }

            // There may be an intersection only if current and next are on different sides of the line:
            if (nextIsLeft != isLeft)  {
                // Compute the distance between the intersection and the begin point of the line.
                // The returned distance should be between 0 (intersection == beginPoint)
                // and 1 (intersection == endPoint):
                float distance = line.IntersectDistance(current, next);
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
        int previousIndex = nextIndex > 0 ? nextIndex - 1 : points.Length - 1;

        Vector2 nextPos = points[nextIndex];
        Vector2 prevPos = points[previousIndex];

        // Rotate (next - prev) to point outside the shape:
        Vector2 normal = Vector2.Perpendicular(prevPos - nextPos);

        // Compute the dot product between n and (circle.position - prevPos).
        // If the dot product is greater than 0, the point is outside the shape:
        bool valid = Vector2.Dot(circle.position - prevPos, normal) >= 0;

        circle.UpdateValidity(valid);
    }

    // Return the index of a point from the shape, with an angle strictly greater
    // than the given point:
    private int FindSection(Vector2 point) {
        float pointAngle = Mathf.Atan2(point.y - center.y, point.x - center.x);

        // Make sure that the angle is greater than the angle of the
        // first point of the shape:
        pointAngle = angles[0] + Mathf.Repeat(pointAngle - angles[0], 2 * Mathf.PI);

        int section = 0;
        while (section < points.Length && angles[section] < pointAngle)
            section++;

        // Keep the result between 0 and points.Count:
        return section == points.Length ? 0 : section;
    }
}