using System;
using UnityEngine;

/// <summary>
/// Serializable class used to represent the parameters for the Kalman Filter
/// </summary>

[Serializable]
public class KalmanParams {
    [Tooltip("If true, writes data of the localisation estimate in the file ./Assets/Data/logs/kalman_estimate.csv")]
    public bool writeLogFile = false;

    [Tooltip("The maximum number of landmarks we can use to update the robot " +
        "state estimate (more landmarks means more precision, but is also slower)")]
    public int maxLandmarksPerUpdate = 5;

    [Tooltip("Minimum distance between landmarks (avoid having too many landmarks at the same place)")]
    public float minDistanceBetweenLandmarks = 1;

    [Tooltip("If true, draws the confirmed landmarks, that are used for localization (in green)")]
    public bool drawConfirmedLandmarks = false;
    [Tooltip("If true, draws the potential landmarks (in red)")]
    public bool drawPotentialLandmarks = false;
    [Tooltip("If true, draws the expected position of the observations used for the localization (in blue)")]
    public bool drawObservations = false;
}