using System;
using UnityEngine.UI;
using UnityEngine;

/// <summary>
/// Serializable class used to represent the parameters used for the mapping using grid maps
/// </summary>

[Serializable]
public class GridMapParams {
    [Tooltip("RawImage from the UI on which the maps will be drawn")]
    public RawImage mapImage;

    [Tooltip("Number of updates to wait before updating the texture (updating the texture is slow)")]
    public int textureUpdateWait = 10;
    [Tooltip("If we should show the RawImage representing the grid maps")]
    public bool showTexture = false;

    [Tooltip("Size of the cells of the grids")]
    public float cellSize = 0.1f;

    [Tooltip("Number of cells along the X-axis")]
    public int mapWidth = 100;
    [Tooltip("Number of cells along the Y-axis")]
    public int mapHeight = 100;

    [Range(0.5f, 1)]
    [Tooltip("Maximum confidence probability of a cell")]
    public float maxConfidence = 0.99f;

    [Tooltip("Average number of frames to wait before considering an object as static in the static grid")]
    public int framesToConsiderStatic = 60;

    [Tooltip("Average number of frames to wait before considering an object as dynamic in the dynamic grid")]
    public int framesToConsiderDynamic = 20;
}