using System;
using UnityEngine;

[Serializable]
public class KalmanParams {
    public bool writeLogFile = false;

    [Tooltip("The maximum number of landmarks we can use to update the robot " +
        "state estimate (more landmarks means more precision, but is also slower)")]
    public int maxLandmarksPerUpdate = 5;

    [Tooltip("Minimum distance between landmarks (avoid having too many landmarks at the same place)")]
    public float minDistanceBetweenLandmarks = 1;

    public bool drawConfirmedLandmarks = false;
    public bool drawPotentialLandmarks = false;
    public bool drawObservations = false;
}