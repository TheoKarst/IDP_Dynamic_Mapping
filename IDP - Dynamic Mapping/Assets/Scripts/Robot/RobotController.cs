using UnityEngine;

public class RobotController {
    // GameObject representing the robot:
    private GameObject robot;

    // List of parameters representing the behaviour of the script:
    private ControllerParams parameters;

    private float velocity = 0;
    private float steering = 0;

    public RobotController(GameObject robot, ControllerParams parameters) {
        this.robot = robot;
        this.parameters = parameters;
    }

    // This should be called once per frame:
    public void Update() {
        float h = Time.deltaTime;

        // Update the velocity of the robot:
        if (parameters.forceForward || Input.GetKey(KeyCode.UpArrow))
            velocity += h * parameters.acceleration;
        else if (parameters.forceBackward || Input.GetKey(KeyCode.DownArrow))
            velocity -= h * parameters.acceleration;
        else
            velocity -= h * velocity * parameters.friction;

        // Clamp the velocity between boundaries:
        velocity = Mathf.Clamp(velocity, -parameters.maxSpeed, parameters.maxSpeed);

        // Update the steering of the robot:
        if (parameters.forceLeft || Input.GetKey(KeyCode.LeftArrow))
            steering = parameters.maxSteering;
        else if (parameters.forceRight || Input.GetKey(KeyCode.RightArrow))
            steering = -parameters.maxSteering;
        else
            steering = 0;

        // Compute the derivative of the position and orientation of the robot:
        float robotAngle = GetRobotAngle();
        float xP = velocity * Mathf.Cos(robotAngle);
        float yP = velocity * Mathf.Sin(robotAngle);
        float angleP = velocity * Mathf.Tan(Mathf.Deg2Rad * steering) / parameters.L;

        // Add random noise to the pose of the robot (to represent a more realistic model):
        float noiseX = Utils.RandomGaussian(0, parameters.noiseX);
        float noiseY = Utils.RandomGaussian(0, parameters.noiseY);
        float noisePhi = Utils.RandomGaussian(0, parameters.noisePhi);

        // Update the position and orientation of the robot, given the previous values:
        robot.transform.position += new Vector3(h * xP + noiseX, 0, h * yP + noiseY);
        robot.transform.Rotate(Vector3.up, -h * angleP * Mathf.Rad2Deg - noisePhi);
    }

    public ModelInputs GetModelInputs() {
        return new ModelInputs(velocity, Mathf.Deg2Rad * steering);
    }

    public VehicleState GetRobotRealState() {
        float x = robot.transform.position.x;
        float y = robot.transform.position.z;
        float angle = GetRobotAngle();

        return new VehicleState(x, y, angle);
    }

    private float GetRobotAngle() {
        return Mathf.Deg2Rad * (90 - robot.transform.rotation.eulerAngles.y);
    }
}
