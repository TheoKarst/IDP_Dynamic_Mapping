using System;
using UnityEngine;

/// <summary>
/// Serializable class used to represent the parameters of the controller of the robot (used
/// to move the robot in Unity scene with the arrow keys of the keyboard)
/// </summary>

[Serializable]
public class ControllerParams {
    [Tooltip("Used for debugging: If true, this will force the robot to move forward " +
        "(same as keeping the up arrow key pressed)")]
    public bool forceForward = false;

    [Tooltip("Used for debugging: If true, this will force the robot to move backward " +
        "(same as keeping the down arrow key pressed)")]
    public bool forceBackward = false;

    [Tooltip("Used for debugging: If true, this will force the robot to turn left " +
        "(same as keeping the left arrow key pressed)")]
    public bool forceLeft = false;

    [Tooltip("Used for debugging: If true, this will force the robot to turn right " +
        "(same as keeping the right arrow key pressed)")]
    public bool forceRight = false;

    [Tooltip("Acceleration of the robot")]
    public float acceleration = 10;

    [Tooltip("Length of the robot: used for rotating")]
    public float L = 0.3f;

    [Tooltip("Maximum speed of the robot")]
    public float maxSpeed = 2f;

    [Tooltip("Maximum steering angle of the front wheels (in degrees)")]
    public float maxSteering = 10;

    [Tooltip("Friction applied to the robot")]
    public float friction = 10;

    [Tooltip("Random noise added to the theoretical X-position of the robot")]
    public float noiseX = 0;
    
    [Tooltip("Random noise added to the theoretical Y-position of the robot")]
    public float noiseY = 0;

    [Tooltip("Random noise added to the theoretical angle of the robot")]
    public float noisePhi = 0;
}