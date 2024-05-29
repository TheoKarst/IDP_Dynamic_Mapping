using System.Collections.Generic;
using UnityEngine;

public class WipeTriangle {
    // The points defining the wipe triangle:
    private Vector2 p1, p2, p3;

    // Normals of the triangle
    private Vector2 n1, n2, n3;

    public WipeTriangle(Vector2 p1, Vector2 p2, Vector2 p3) {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;

        n1 = Normal(p1, p2);
        n2 = Normal(p2, p3);
        n3 = Normal(p3, p1);
    }

    public List<Line> UpdateLines(List<Line> lines, float minLineLength) {
        List<Line> result = new List<Line>();

        foreach(Line line in lines) {
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
                    result.Add(line);
            }
            else if(p2_inside) {
                if (i1 != null) line.endPoint = i1.position;
                else if (i2 != null) line.endPoint = i2.position;
                else if (i3 != null) line.endPoint = i3.position;

                if (line.Length() >= minLineLength)
                    result.Add(line);
            }

            // If both points are outside of the triangle, and there is no intersection
            // between the line and the triangle, then the line is completely outside of
            // the triangle and should remain unaffected:
            else if(i1 == null && i2 == null && i3 == null) {
                result.Add(line);
            }

            // Otherwise, we will have to split the line in two:
            else {
                Intersection first = Intersection.First(i1, i2, i3);
                Intersection last = Intersection.Last(i1, i2, i3);

                line.endPoint = first.position;
                if(line.Length() >= minLineLength) result.Add(line);

                Line other = new Line(line, last.position, line.endPoint);
                if(other.Length() >= minLineLength) result.Add(other);
            }
        }

        return result;
    }

    // Return if the given point is inside this triangle:
    private bool Contains(Vector2 point) {
        return Vector2.Dot(point - p1, n1) <= 0
            && Vector2.Dot(point - p2, n2) <= 0
            && Vector2.Dot(point - p3, n3) <= 0;
    }

    // Compute the normal unit vector of the given segment:
    private Vector2 Normal(Vector2 A, Vector2 B) {
        Vector2 n = (B - A).normalized;
        
        // Rotate the vector:
        n.Set(-n.y, n.x);

        return n;
    }
}
