using System;
using UnityEngine.UI;
using UnityEngine;

[Serializable]
public class GridMapParams {
    public RawImage mapImage;

    // Number of updates to wait before updating the texture:
    public int textureUpdateWait = 10;
    public bool showTexture = false;

    public float cellSize = 0.1f;
    public int mapWidth = 100;
    public int mapHeight = 100;

    public float maxConfidence = 0.99f;

    [Tooltip("Average number of frames to wait before considering an object as static in the static grid")]
    public int framesToConsiderStatic = 60;

    [Tooltip("Average number of frames to wait before considering an object as dynamic in the dynamic grid")]
    public int framesToConsiderDynamic = 20;
}