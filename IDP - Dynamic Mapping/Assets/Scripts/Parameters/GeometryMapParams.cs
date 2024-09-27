using MathNet.Numerics.LinearAlgebra;
using System;
using UnityEngine;

/// <summary>
/// Serializable class used to represent the parameters used for the mapping using geometric primitives (lines and circles)
/// </summary>

[Serializable]
public class GeometryMapParams {
    /***********************************************************************************************/
    // This section describes the parameter of the grid map used to speed up the matching between
    // observed objects (circles and lines), and objects in the world model:
    
    [Header("Line match grid map")]
    
    public float centerX = 0;
    public float centerY = 0;
    public int width = 40;
    public int height = 40;
    public float cellSize = 2;

    /***********************************************************************************************/
    // This section describes the parameters used during the extraction of primitives (lines, circles)
    // from observed points:

    [Header("Geometry Extraction:")]

    [Tooltip("Maximum distance between two consecutive points to be matched to the same line")]
    public float PointCriticalDistance = 0.2f;

    [Tooltip("Maximum orthogonal distance of a point to a matching line")]
    public float LineCriticalDistance = 0.06f;

    [Tooltip("Maximum angle (in degrees) between two consecutive points to be matched to the same line")]
    public float CriticalAlpha = 1f;

    [Tooltip("Maximum distance between a point and the center of a circle to match the point to that circle")]
    public float CircleCriticalDistance = 0.2f;

    [Tooltip("Minimum length of a line (smaller lines are considered as circle clusters)")]
    public float LineMinLength = 0.3f;

    [Tooltip("Minimum number of points a line should have to be extracted")]
    [Min(3)]
    public int LineMinPoints = 6;

    [Tooltip("Minimum number of points a circle should have to be extracted")]
    [Min(2)]
    public int CircleMinPoints = 6;

    /***********************************************************************************************/
    // This section describes the parameters used for the matching between observed primitives and
    // primitives in the model:

    [Header("Geometry Matching:")]

    [Tooltip("Maximum angle (in degrees) between the current line "
        + "and the model line to have a match")]
    public float LineMaxMatchAngle = 10f;

    [Tooltip("Maximum orthogonal distance between the current "
        + "line endpoints and the model line to have a match")]
    public float LineMaxMatchOrthogonalDistance = 0.1f;

    [Tooltip("Maximum distance between the current line and the "
        + "model line endpoints, along the model line, to have a match")]
    public float LineMaxMatchParallelDistance = 0.2f;

    [Tooltip("Maximum distance between two circles to be matched together")]
    public float CircleMaxMatchDistance = 0.2f;

    [Tooltip("Parameter used to extend the validity of a line during matching")]
    public float LineValidityExtent = 0.1f;

    /***********************************************************************************************/
    // This section describes parameters used for dynamic lines:

    [Header("Dynamic Lines")]
    [Tooltip("If this is true, the static Kalman Filter is used to update the lines."
        + "Otherwise, we use a dynamic Kalman Filter (taking into account the lines speed)")]
    public bool StaticLines = false;

    [Tooltip("If true, data about the lines in the model will be recorded in the file ./Assets/Data/logs/lines_data.csv")]
    public bool LogLinesData = false;

    [Tooltip("Number of matches to consider a line completely initialised")]
    public int InitialisationSteps = 10;

    [Tooltip("Maximum error estimate on the range of a line to keep it")]
    public float MaxLinesRhoError = 1;
    [Tooltip("Maximum error on the angle (in degrees) of a line to keep it")]
    public float MaxLinesThetaError = 10;

    [Tooltip("If a line is matched this amount of times, having a speed below the following thresholds, "
        + "then we consider it to be static, and start to ignore the prediction step of the Kalman Filter")]
    public int MinMatchesToConsiderStatic = 10;

    [Tooltip("Maximum speed of a static line")]
    public float StaticMaxRhoDerivative = 0.1f;

    [Tooltip("Maximum rotation speed of a static line")]
    public float StaticMaxThetaDerivative = 5;

    [Range(0, 1)]
    [Tooltip("Friction applied to the speed of the lines")]
    public float LinesFriction = 0.01f;
    public float LineProcessNoiseRho = 0.1f;
    public float LineProcessNoiseTheta = 5;
    public float LineProcessNoiseDerRho = 1;
    public float LineProcessNoiseDerTheta = 50;

    /***********************************************************************************************/
    // This section describes parameters used for circles:

    [Header("Dynamic Circles")]
    [Tooltip("Minimum allowed distance between a circle and a line")]
    public float MinOrthogonalDistanceToLines = 1;
    [Range(0, 1)]
    [Tooltip("Friction applied to the speed of the circles")]
    public float CirclesFriction = 0.01f;

    /***********************************************************************************************/
    // This section describes parameters used for the construction of the wipe-shape:

    [Header("Wipe Shape")]

    [Range(0f, 90f)]
    [Tooltip("Angle in degrees of the wipe angle associated to the observations")]
    public float alpha = 15;
    [Min(0f)]
    [Tooltip("Epsilon used in Douglas Peucker algorithm to filter the points of the shape")]
    public float epsilon = 0.2f;
    [Min(0)]
    [Tooltip("Clamp the observation distances to build the wipe shape")]
    public float clampDistance = 10;

    /***********************************************************************************************/
    // This section describes which elements to draw in the scene:

    [Header("Drawing")]

    [Tooltip("If true, draws the grid used for the faster matching of primitives")]
    public bool drawMatchGrid = false;
    [Tooltip("If true, draws the current speed estimate of the primitives in the model")]
    public bool drawSpeedEstimates = true;
    [Tooltip("If true, draws the points observed by the LIDAR")]
    public bool drawPoints = true;
    [Tooltip("If true, draws for each point a cube representing the error estimate for that point")]
    public bool drawPointsError = true;
    [Tooltip("If true, draws the current lines in the model")]
    public bool drawLines = true;
    [Tooltip("If true, draws the current circles in the model")]
    public bool drawCircles = true;
    [Tooltip("If true, draws the lines that are currently observed (used for debugging)")]
    public bool drawCurrentLines = true;
    [Tooltip("If true, draws the wipe-shape representing the free space of the sensor")]
    public bool drawWipeShape = true;

    /***********************************************************************************************/
    // This section contains some useful functions to define variables from the above parameters:

    private Matrix<double> _LineProcessNoiseError;

    // Values computed from the previous parameters:
    public float CriticalAlphaRadians { get => Mathf.Deg2Rad * CriticalAlpha; }
    public float LineMaxMatchAngleRadians { get => Mathf.Deg2Rad * LineMaxMatchAngle; }
    public float MaxLineErrorRhoSq { get => MaxLinesRhoError * MaxLinesRhoError; }
    public float MaxLineErrorThetaSq { get => MaxLinesThetaError * MaxLinesThetaError 
                                            * Mathf.Deg2Rad * Mathf.Deg2Rad; }
    public float StaticMaxThetaDerivativeRadians { get => StaticMaxThetaDerivative * Mathf.Deg2Rad; }


    public Matrix<double> LineProcessNoiseError { get => ComputeLineProcessNoiseError(); }

    private Matrix<double> ComputeLineProcessNoiseError() {
        // if (_LineProcessNoiseError == null) {
            float eRho = LineProcessNoiseRho;
            float eTheta = Mathf.Deg2Rad * LineProcessNoiseTheta;
            float eDerRho = LineProcessNoiseDerRho;
            float eDerTheta = Mathf.Deg2Rad * LineProcessNoiseDerTheta;

            _LineProcessNoiseError = Matrix<double>.Build.Diagonal(new double[] {
                eRho * eRho,
                eTheta * eTheta,
                eDerRho * eDerRho,
                eDerTheta * eDerTheta
            });
        // }

        return _LineProcessNoiseError;
    }
}