using UnityEngine;

class Point {
    public float x;
    public float y;
    public float angle;

    public Point(float x, float y, float angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public static float dist(Point a, Point b) {
        float dX = a.x - b.x;
        float dY = a.y - b.y;

        return Mathf.Sqrt(dX * dX + dY * dY);
    }

    public static float angularDifference(Point a, Point b) {
        return Mathf.Abs(a.angle - b.angle);
    }
}