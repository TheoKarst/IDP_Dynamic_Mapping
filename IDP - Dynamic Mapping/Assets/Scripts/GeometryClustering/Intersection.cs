using UnityEngine;

public class Intersection {
    public Vector2 position;
    public float distance;

    public Intersection(Vector2 position, float distance) {
        this.position = position;
        this.distance = distance;
    }

    public static Intersection First(params Intersection[] intersections) {
        Intersection first = null;
        float minDistance = -1;

        foreach (Intersection intersection in intersections) {
            if (intersection != null && (first == null || intersection.distance < minDistance)) {
                first = intersection;
                minDistance = intersection.distance;
            }
        }

        return first;
    }

    public static Intersection Last(params Intersection[] intersections) {
        Intersection last = null;
        float maxDistance = -1;

        foreach (Intersection intersection in intersections) {
            if (intersection != null && (last == null || intersection.distance > maxDistance)) {
                last = intersection;
                maxDistance = intersection.distance;
            }
        }

        return last;
    }
}