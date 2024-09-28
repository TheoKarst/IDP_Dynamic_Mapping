using System;
using UnityEngine;

/// <summary>
/// Serializable class used to represent the parameters of a LIDAR
/// </summary>

[Serializable]
public class LidarParams {
    [Tooltip("Number of rays of the LIDAR")]
    public int raycastCount = 500;

    [Tooltip("Minimum range of the LIDAR (rays below that range are deleted)")]
    public float lidarMinRange = 0;

    [Tooltip("Maximum range of the LIDAR (rays above that range are clamped and marked as \"out of bounds\"")]
    public float lidarMaxRange = 50f;

    [Tooltip("If true, draw the rays of the LIDAR in the scene")]
    public bool drawRays = true;
}