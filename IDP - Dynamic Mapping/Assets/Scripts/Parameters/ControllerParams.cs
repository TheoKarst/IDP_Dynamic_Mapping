using System;

/// <summary>
/// Serializable class used to represent the parameters of the controller of the robot (used
/// to move the robot in Unity scene with the arrow keys of the keyboard)
/// </summary>

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

    public float noiseX = 0;
    public float noiseY = 0;
    public float noisePhi = 0;
}