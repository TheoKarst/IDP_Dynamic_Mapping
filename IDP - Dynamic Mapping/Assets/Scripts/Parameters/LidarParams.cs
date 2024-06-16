using System;

[Serializable]
public class LidarParams {
    public int raycastCount = 500;
    public float raycastDistance = 10;
    public float douglasPeuckerEpsilon = 0.2f;

    public bool drawRays = true;
    public bool drawCorners = true;
}