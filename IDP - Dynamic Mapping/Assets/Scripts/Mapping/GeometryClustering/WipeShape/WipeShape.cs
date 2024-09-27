using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

/// <summary>
/// The WipeShape is a polygon representing the free space of a sensor. This is used to remove primitives
/// inconsistent with the current observations
/// </summary>
public class WipeShape {
    
    // Center of the shape, which is supposed to be a "radial set" (any line from the center to a
    // point in the following list lies in the shape):
    private Vector2 center;

    // World space position of the points of the shape:
    private Vector2[] points;

    // World space angles of the points of the shape from the origin.
    // The list is supposed to be sorted, with angles[last] - angles[first] < 2*PI:
    private float[] angles;

    /// <summary>
    /// Instantiates a wipe-shape, that is a polygon representing the free space around the sensor
    /// </summary>
    /// <param name="center">Point inside the polygon, such that each line between the center and 
    /// a point of the polygon should belong to the polygon. The polygon is thus supposed to be a 
    /// "radial set", which is the case if the center is the position of the LIDAR</param>
    /// <param name="points">Positions of the points of the polygon</param>
    /// <param name="angles">Angles of the rays between the center and the positions</param>
    public WipeShape(Vector2 center, Vector2[] points, float[] angles) {
        this.center = center;
        this.points = points;
        this.angles = angles;

        for (int i = 1; i < angles.Length; i++)
            Assert.IsTrue(angles[i - 1] < angles[i], "Angles order error !");

        if (angles[angles.Length - 1] - angles[0] > 2 * Mathf.PI)
            Debug.LogError("Angles > 2*PI: " + Utils.ToString(angles, Mathf.Rad2Deg) + "]; First point: " + points[0] + ", Last: " + points[points.Length - 1]);

    }

    /// <summary>
    /// Draws the wipe-shape in the scene, using Unity Gizmos
    /// </summary>
    /// <param name="height">Position of the wipe-shape along the Z-axis</param>
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

    /// <summary>
    /// Updates which part of the given line are valid or invalid, knowing that 
    /// all the parts overlapping the wipe-shape should be invalid
    /// </summary>
    public void UpdateLineValidity(DynamicLine line) {
        if (points.Length < 3) {
            Debug.LogError("Wipe Shape has less than 3 points !");
            return;
        }

        // All the indices are modulo n:
        int n = points.Length;

        int startSection, endSection, direction, count;

        // Compute if we have to check the intersections between the edges of the
        // shape and the line in clockwise or counter-clockwise order:
        if (line.SideOfPoint(center) >= 0) {
            // Turn clockwise:
            direction = -1;

            startSection = FindSectionUpper(line.beginPoint);
            endSection = FindSectionLower(line.endPoint);
            if (endSection < 0) endSection += n;
            count = startSection - endSection;
        }
        else {
            // Turn counter-clockwise:
            direction = 1;

            startSection = FindSectionLower(line.beginPoint);
            endSection = FindSectionUpper(line.endPoint);
            if (startSection < 0) startSection += n;
            count = endSection - startSection;
        }

        // We have startPoint in [0; n[ and endPoint in [0; n[. Thus count is in ]-n; n[. We just have to keep count in ]0; n]:
        if (count <= 0) count += n;

        int index = startSection;
        Vector2 current = points[index];
        bool isRight = line.SideOfPoint(current) >= 0;

        // If the start point of the line is outside the shape, and if the current
        // side of the points of the shape compared to the line is known or not.
        // The side is said unknown if the points belongs to the line. The initial values
        // doesn't matter since they will be replaced during the first loop:
        bool startPointOutside = true;
        bool currentSideUnknown = true;

        string logMsg = "";

        List<float> intersections = new List<float>();
        for (int i = 0; i < count; i++) {
            index += direction;
            if (index < 0) index += n;
            else if (index >= n) index -= n;

            Vector2 next = points[index];
            int nextLineSide = line.SideOfPoint(next);
            
            // Here we manage the situation where the point is exactly on the line.
            // In this case, we keep the same state as before:
            bool nextIsRight = nextLineSide == 0 ? isRight : nextLineSide > 0;

            logMsg += "Next line distance of point: " + next + " = " + Vector2.Dot(Vector2.Perpendicular(line.beginPoint - line.endPoint), next - line.beginPoint)
                + " => Side: " + nextLineSide + "(NextIsRight: " + nextIsRight + ", IsRight: " + isRight + "\n";

            // Compute startPointOutside:
            if (i == 0) {
                // Rotate (next - current) to point outside the shape:
                Vector2 normal = direction * Vector2.Perpendicular(current - next);

                // Compute the dot product between (line.beginPoint - current) and the normal:
                float dotN = Vector2.Dot(line.beginPoint - current, normal);

                // If the dot product is strictly greater than 0, the point is outside the shape:
                startPointOutside = dotN > 0;

                // But if the dot product equals zero (this can happen if line.start == current),
                // the point is on the line, and we mark the current side as unknown:
                currentSideUnknown = dotN == 0;
            }

            // There may be an intersection only if current and next are on different sides of the line:
            if (nextIsRight != isRight && !currentSideUnknown) {
                // Compute the distance between the intersection and the begin point of the line.
                // The returned distance should be between 0 (intersection == beginPoint)
                // and 1 (intersection == endPoint):
                float distance = line.IntersectDistance(current, next);
                if (distance > 0 && distance < 1) {

                    // The distance between two changes should be greater than 0.01:
                    int last = intersections.Count - 1;
                    if(last >= 0 && distance - intersections[last] < 0.01)
                        intersections.RemoveAt(last);
                    else
                        intersections.Add(distance);

                    logMsg += "Intersection at " + distance + ", with line [" + current + "; " + next + "]\n";
                }
            }

            // If all the previous points were exactly on the line, but not the next point,
            // we can update the value of startPointOutside and currentSideUnknown:
            if (currentSideUnknown && nextLineSide != 0) {
                // If direction == -1, we know the center of the shape is on the right of the
                // line, and if nextLineDistance > 0, we know that the next point is also on
                // the right of the line. In this situation, the line is going outside of the
                // shape, so startPointOutside = true.
                // This is also the case if direction == 1 and nextLineDistance < 0:
                startPointOutside = (direction == -1 && nextLineSide > 0) 
                                 || (direction == 1 && nextLineSide < 0);
                currentSideUnknown = false;
            }

            current = next;
            isRight = nextIsRight;
        }

        if(intersections.Count == 0) {
            Debug.Log("No intersections between line [" + line + "] and wipe shape. "
                + "Start section: " + startSection
                + ", End section: " + endSection
                + ", Direction: " + direction
                + ", Count: " + count
                + ", Start outside: " + startPointOutside
                + (currentSideUnknown ? " (unknown)" : " (known)"));
        }

        // intersections should be a sorted list:
        bool sorted = true;
        for(int i = 1; i < intersections.Count; i++)
            if (intersections[i] <= intersections[i-1])
                sorted = false;

        if(!sorted) {
            Debug.LogError("The list of intersections is not sorted !"
                + ", Line: " + line
                + "\nStart section: " + startSection
                + ", End section: " + endSection
                + ", Direction: " + direction
                + ", Count: " + count
                + ", Start outside: " + startPointOutside
                + (currentSideUnknown ? " (unknown)" : " (known)")
                + ", Intersections:\n" + logMsg);
        }

        line.ResetLineValidity(startPointOutside, intersections);
    }

    /// <summary>
    /// Sets the validity of the given circle to False if its center is 
    /// inside the wipe-shape and True otherwise
    /// </summary>
    public void UpdateCircleValidity(Circle circle) {
        int nextIndex = FindSectionUpper(circle.center);
        int previousIndex = nextIndex > 0 ? nextIndex - 1 : points.Length - 1;

        Vector2 nextPos = points[nextIndex];
        Vector2 prevPos = points[previousIndex];

        // Rotate (next - prev) to point outside the shape:
        Vector2 normal = Vector2.Perpendicular(prevPos - nextPos);

        // Compute the dot product between n and (circle.position - prevPos).
        // If the dot product is greater than 0, the point is outside the shape:
        circle.isValid = Vector2.Dot(circle.center - prevPos, normal) > 0;
    }

    /// <summary>
    /// Returns the index of a point from the shape, with an angle greater or equal to the given point
    /// </summary>
    private int FindSectionUpper(Vector2 point) {
        float pointAngle = Mathf.Atan2(point.y - center.y, point.x - center.x);

        // Make sure that the angle is greater than the angle of the
        // first point of the shape:
        pointAngle = angles[0] + Mathf.Repeat(pointAngle - angles[0], 2 * Mathf.PI);

        int section = 0;
        while (section < angles.Length && angles[section] < pointAngle)
            section++;

        return section == angles.Length ? 0 : section;
    }

    /// <summary>
    /// Returns the index of a point with an angle less or equal to the given point
    /// </summary>
    private int FindSectionLower(Vector2 point) {
        float pointAngle = Mathf.Atan2(point.y - center.y, point.x - center.x);

        // Make sure that the angle is greater than the angle of the
        // first point of the shape:
        pointAngle = angles[0] + Mathf.Repeat(pointAngle - angles[0], 2 * Mathf.PI);

        int section = angles.Length - 1;
        while (section >= 0 && angles[section] > pointAngle)
            section--;

        return section == -1 ? angles.Length - 1 : section;
    }
}