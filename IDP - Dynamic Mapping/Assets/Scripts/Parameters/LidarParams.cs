using System;
using UnityEngine;

/// <summary>
/// Serializable class used to represent the parameters of a LIDAR
/// </summary>

[Serializable]
public class LidarParams {
    [Tooltip("Number of rays of the LIDAR")]
    public int raycastCount = 500;

    [Tooltip("Maximum distance of the rays")]
    public float raycastDistance = 10;

    [Tooltip("Parameter used for the filtering of observations during the localization estimate with the Kalman Filter")]
    public float douglasPeuckerEpsilon = 0.2f;

    [Tooltip("If true, draw the rays of the LIDAR in the scene")]
    public bool drawRays = true;
}