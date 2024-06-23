using System;
using UnityEngine;

[Serializable]
public class GeometryClusterParams {
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

    [Tooltip("Minimum number of points a line should hve to be extracted")]
    [Min(3)]
    public int LineMinPoints = 6;

    [Tooltip("Minimum number of points a circle should have to be extracted")]
    [Min(2)]
    public int CircleMinPoints = 6;

    [Header("Geometry Matching:")]
    [Tooltip("Maximum angle (in degrees) between the current line "
        + "and the model line to have a match")]
    public float LineMaxMatchAngle = 10f;

    [Tooltip("Maximum orthogonal distance between the current "
        + "line endpoints and the model line to have a match")]
    public float LineMaxEndpointMatchDistance = 0.1f;

    [Tooltip("Maximum distance between the current line and the "
        + "model line centers to have a match")]
    public float LineMaxMatchDistance = 0.2f;

    [Tooltip("Maximum distance between two circles to be matched together")]
    public float CircleMaxMatchDistance = 0.2f;

    // Deprecated:
    // [Header("Geometry removing")]
    // [Tooltip("Extent of the wipe triangle built for new lines")]
    // public float WipeTriangleExtent = 0.1f;
    // public float WipeShapeExtent = 0.1f;

    // [Tooltip("Margin angle (in degrees) of the wipe triangles")]
    // public float WipeTriangleInsideAngleMargin = 1f;

    // [Tooltip("Minimum distance between a circle and a line in the model")]
    // public float MinCircleLineDistance = 0.5f;

    [Header("Wipe Shape")]
    [Range(0f, 90f)]
    [Tooltip("Angle of the wipe cone associated to the observations")]
    public float alpha = 15;
    [Min(0f)]
    [Tooltip("Epsilon used in Douglas Peucker algorithm to filter the points of the shape")]
    public float epsilon = 0.2f;
    [Min(0)]
    [Tooltip("Clamp the observation distances to build the wipe shape")]
    public float clampDistance = 10;

    [Header("Drawing")]
    public bool drawPoints = true;
    public bool drawLines = true;
    public bool drawCircles = true;
    public bool drawCurrentLines = true;
    public bool drawWipeShape = true;
}