using System;

[Serializable]
public class ControllerParams {
    public bool forceForward = false;
    public bool forceBackward = false;
    public bool forceLeft = false;
    public bool forceRight = false;

    public float acceleration = 10;
    public float L = 0.3f;

    public float maxSpeed = 2f;
    public float maxSteering = 10;

    public float friction = 10;
}