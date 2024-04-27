using UnityEngine;
using UnityEngine.UIElements;

public class Lidar : MonoBehaviour
{
    public int raycastCount = 20;
    public float raycastDistance = 1;
    public Transform[] landmarks;

    private float[] raycastDistances;

    // Start is called before the first frame update
    void Start() {
        raycastDistances = new float[raycastCount];
    }

    // Update is called once per frame
    void Update() {
        // Update the raycast distances each frame:
        Vector3 direction = transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < raycastDistances.Length; i++) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, raycastDistance)) {
                // Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                raycastDistances[i] = hit.distance;
            }
            else {
                // Debug.DrawRay(transform.position, direction * raycastDistance, Color.white);
                raycastDistances[i] = -1;
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
        }
    }

    public float[] getRaycastDistances() {
        return raycastDistances;
    }

    public float getLidarA() {
        return transform.localPosition.z;
    }

    public float getLidarB() {
        return -transform.localPosition.x;
    }

    // To update the state of the robot, we should get one landmark observation each frame.
    // For now, we suppose that the sensor returns a random landmark, no matter its distance from the robot:
    public (float, float) getLandmark() {

        // Select a random landmark:
        Transform landmark = landmarks[Random.Range(0, landmarks.Length)];

        float dX = landmark.position.x - transform.position.x;
        float dY = landmark.position.z - transform.position.z;
        float distance = Mathf.Sqrt(dX * dX + dY * dY);

        float lidarAngle = Mathf.Deg2Rad * (90 - gameObject.transform.rotation.eulerAngles.y);
        float angle = Mathf.Atan2(dY, dX) - lidarAngle;

        return (distance, angle);
    }

    public void DrawObservation(Observation observation, Color color) {
        float duration = 0.05f;

        Vector3 direction = transform.TransformDirection(Vector3.forward);
        direction = Quaternion.AngleAxis(-observation.theta * Mathf.Rad2Deg, Vector3.up) * direction;

        Debug.DrawRay(transform.position, direction * observation.r, color, duration);
    }
}
