using System.Collections.Generic;
using UnityEngine;

public class WipeTriangle {
    /*// The points defining the wipe triangle:
    private Vector2 p1, p2, p3;

    // Normals of the triangle (not normalized, since its not necessary):
    private Vector2 n1, n2, n3;

    public WipeTriangle(Vector2 p1, Vector2 p2, Vector2 p3) {
        // The points of the triangle are supposed to be in counter clockwise order:
        if (Vector2.Dot(RotateClockwise(p1, p2), p3 - p2) >= 0)
            Debug.LogError("Wrong set of points to construct wipe triangle");

        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;

        n1 = RotateClockwise(p1, p2);
        n2 = RotateClockwise(p2, p3);
        n3 = RotateClockwise(p3, p1);
    }

    // Update the given lines using this wipe triangle. If the line 'ignore' is found in the given list,
    // the line is not affected and not added in the result. If ignore == null, no line is ignored.
    // dest: The list in which the result will be added
    public void UpdateLines(List<Line> lines, Line ignore, float minLineLength, List<Line> dest) {
        dest.Clear();

        foreach(Line line in lines) {
            if(line == ignore)
                continue;

            bool p1_inside = Contains(line.beginPoint);
            bool p2_inside = Contains(line.endPoint);

            // If the line is included in the triangle, delete the line:
            if (p1_inside && p2_inside)
                continue;

            // Compute the intersections between the line and this triangle
            Intersection i1 = line.ComputeIntersection(p1, p2);
            Intersection i2 = line.ComputeIntersection(p2, p3);
            Intersection i3 = line.ComputeIntersection(p3, p1);

            // If only one point of the line is inside the triangle, we should shorten the line:
            if(p1_inside) {
                if (i1 != null) line.beginPoint = i1.position;
                else if (i2 != null) line.beginPoint = i2.position;
                else if (i3 != null) line.beginPoint = i3.position;

                if (line.Length() >= minLineLength)
                    dest.Add(line);
            }
            else if(p2_inside) {
                if (i1 != null) line.endPoint = i1.position;
                else if (i2 != null) line.endPoint = i2.position;
                else if (i3 != null) line.endPoint = i3.position;

                if (line.Length() >= minLineLength)
                    dest.Add(line);
            }

            // If both points are outside of the triangle, and there is no intersection
            // between the line and the triangle, then the line is completely outside of
            // the triangle and should remain unaffected:
            else if(i1 == null && i2 == null && i3 == null) {
                dest.Add(line);
            }

            // Otherwise, we will have to split the line in two:
            else {
                Intersection first = Intersection.First(i1, i2, i3);
                Intersection last = Intersection.Last(i1, i2, i3);

                // Create a new line, from last.position to line.endPoint:
                Line other = new Line(line, last.position, line.endPoint);
                if (other.Length() >= minLineLength) dest.Add(other);

                // Shorten the initial line, from line.beginPoint to first.position:
                line.endPoint = first.position;
                if(line.Length() >= minLineLength) dest.Add(line);
            }
        }
    }

    // Update the given lines using this wipe triangle. If the line 'ignore' is found in the given list,
    // the line is left unchanged in the list. If ignore == null, no line is ignored.
    public void UpdateLines(LinkedList<Line> lines, Line ignore, float minLineLength) {
        if (lines.Count == 0)
            return;

        LinkedListNode<Line> currentNode = lines.First;

        while(currentNode != null) {
            Line line = currentNode.Value;

            if (line == ignore || IsClearlyOutside(line)) {
                currentNode = currentNode.Next;
                continue;
            }

            bool p1_inside = Contains(line.beginPoint);
            bool p2_inside = Contains(line.endPoint);

            // If the line is included in the triangle, delete the line:
            if (p1_inside && p2_inside) {
                lines.Remove(currentNode);
                currentNode = currentNode.Next;
                continue;
            }

            // Compute the intersections between the line and this triangle
            Intersection i1 = line.ComputeIntersection(p1, p2);
            Intersection i2 = line.ComputeIntersection(p2, p3);
            Intersection i3 = line.ComputeIntersection(p3, p1);

            // If only one point of the line is inside the triangle, we should shorten the line:
            if (p1_inside) {
                if (i1 != null) line.beginPoint = i1.position;
                else if (i2 != null) line.beginPoint = i2.position;
                else if (i3 != null) line.beginPoint = i3.position;

                // If the line is now too small, remove it:
                if (line.Length() < minLineLength)
                    lines.Remove(currentNode);
            }
            else if (p2_inside) {
                if (i1 != null) line.endPoint = i1.position;
                else if (i2 != null) line.endPoint = i2.position;
                else if (i3 != null) line.endPoint = i3.position;

                // If the line is now too small, remove it:
                if (line.Length() < minLineLength)
                    lines.Remove(currentNode);
            }

            // If both points are outside of the triangle, and there is no intersection
            // between the line and the triangle, then the line is completely outside of
            // the triangle and should remain unaffected. Otherwise, we will have to split
            // the line in two:
            else if (i1 != null || i2 != null || i3 != null) {
                Vector2 tmp = line.endPoint;
                Intersection first = Intersection.First(i1, i2, i3);
                Intersection last = Intersection.Last(i1, i2, i3);

                // Shorten the initial line, from line.beginPoint to first.position:
                line.endPoint = first.position;

                // If the line is now too small, remove it:
                if (line.Length() < minLineLength)
                    lines.Remove(currentNode);

                // Create a new line, from last.position to line.endPoint:
                Line other = new Line(line, last.position, tmp);
                if (other.Length() >= minLineLength) {
                    if(currentNode.Next == null)
                        currentNode = lines.AddLast(other);
                    else
                        currentNode = lines.AddBefore(currentNode.Next, other);
                }
            }

            currentNode = currentNode.Next;
        }
    }

    // Fast check to detect if there is a possibility for the given line to intersect this triangle:
    private bool IsClearlyOutside(Line line) {
        float d1 = line.SignedDistanceFrom(p1);
        float d2 = line.SignedDistanceFrom(p2);
        float d3 = line.SignedDistanceFrom(p3);

        // If all the points are on the same side of the line, then there is no intersection for sure:
        if((d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0)) {
            return true;
        }

        return false;
    }

    // Remove from the list all circles inside this wipe triangle:
    public List<Circle> UpdateCircles(List<Circle> circles) {
        List<Circle> result = new List<Circle>();

        foreach(Circle circle in circles) {
            if(!Contains(circle.position))
                result.Add(circle);
        }

        return result;
    }

    // Return if the given point is inside this triangle:
    private bool Contains(Vector2 point) {
        return Vector2.Dot(point - p1, n1) <= 0
            && Vector2.Dot(point - p2, n2) <= 0
            && Vector2.Dot(point - p3, n3) <= 0;
    }

    // Rotate the segment AB of 90° in clockwise order:
    private Vector2 RotateClockwise(Vector2 A, Vector2 B) {
        return new Vector2(B.y - A.y, A.x - B.x);
    }*/
}
