using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour
{
    public struct Corner {
        public Vector3 position;
        public float angle;

        public Corner(Vector3 position, float angle) {
            this.position = position;
            this.angle = angle;
        }
    }

    public int raycastCount = 20;
    public float raycastDistance = 1;

    // For each measurement i of the LIDAR, we check if it's a corner by comparing it with the measure
    // i + deltaCorners and the measure i - deltaCorners:
    public int deltaCorners = 1;

    // The minimum angle for a corner to be detected as such:
    public float angleThreshold = 80;

    public bool drawRays = true;

    private float[] raycastDistances;
    private Vector3[] hitPoints;
    private List<Corner> corners;

    // Start is called before the first frame update
    void Start() {
        raycastDistances = new float[raycastCount];
        hitPoints = new Vector3[raycastCount];
    }

    // Update is called once per frame
    void Update() {
        // Update the raycast distances each frame:
        Vector3 direction = transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < raycastDistances.Length; i++) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, raycastDistance)) {
                if(drawRays) Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                raycastDistances[i] = hit.distance;
                hitPoints[i] = hit.point;
            }
            else {
                if(drawRays) Debug.DrawRay(transform.position, direction * raycastDistance, Color.white);
                raycastDistances[i] = -1;
                hitPoints[i] = Vector3.zero;
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
        }

        corners = detectCorners();
    }

    public void OnDrawGizmos() {
        if (corners == null)
            return;

        foreach(Corner corner in corners) {
            Gizmos.color = corner.angle < 180 ? Color.green : Color.red;
            Gizmos.DrawSphere(corner.position, 0.1f);
        }
    }

    private List<Corner> detectCorners() {
        if (raycastDistances == null)
            return null;

        List<Corner> corners = new List<Corner>();
        
        int count = raycastDistances.Length;

        // Use the same naming convention as the paper "Corner Detection for Room Mapping of Fire Fighting Robot":
        for(int i = 0; i < count; i++) {
            int prev = (i + count - deltaCorners) % count;
            int next = (i + deltaCorners) % count;

            float b = raycastDistances[prev];
            float c = raycastDistances[i];
            float e = raycastDistances[next];

            // If any of the measurement found no obstacle, then ignore this point:
            if (b < 0 || c < 0 || e < 0)
                continue;

            // Else, detect if this is a corner using cosine law:
            float a = (hitPoints[i] - hitPoints[prev]).magnitude;
            float d = (hitPoints[i] - hitPoints[next]).magnitude;

            float angleB = Mathf.Acos((a*a + c*c - b*b) / (2*a*c));
            float angleE = Mathf.Acos((d*d + c*c - e*e) / (2*d*c));

            // Angle in degrees of the corner:
            float theta = Mathf.Rad2Deg * (angleB + angleE);

            if(theta < 140 || theta > 220)
                corners.Add(new Corner(hitPoints[i], theta));
        }

        return corners;
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

    /*// To update the state of the robot, we should get one landmark observation each frame.
    // For now, we suppose that the sensor returns a random corner, no matter its distance from the robot:
    public (float, float) getLandmark() {

        // Select a random corner:
        Corner corner = corners[Random.Range(0, corners.Count)];

        float dX = corner.position.x - transform.position.x;
        float dY = corner.position.z - transform.position.z;
        float distance = Mathf.Sqrt(dX * dX + dY * dY);

        float lidarAngle = Mathf.Deg2Rad * (90 - gameObject.transform.rotation.eulerAngles.y);
        float angle = Mathf.Atan2(dY, dX) - lidarAngle;

        return (distance, angle);
    }*/

    // Return the set of all possible landmarks that could be used to update our state estimate:
    public Observation[] getObservations() {
        float lidarAngle = Mathf.Deg2Rad * (90 - gameObject.transform.rotation.eulerAngles.y);
        Observation[] observations = new Observation[corners.Count];
        
        for (int i = 0; i < observations.Length; i++) {
            float dX = corners[i].position.x - transform.position.x;
            float dY = corners[i].position.z - transform.position.z;
            float r = Mathf.Sqrt(dX * dX + dY * dY);
            float theta = Mathf.Atan2(dY, dX) - lidarAngle;

            observations[i] =  new Observation(r, theta);
        }

        return observations;
    }

    public void DrawObservation(Observation observation, Color color) {
        float duration = 0.1f;

        Vector3 direction = transform.TransformDirection(Vector3.forward);
        direction = Quaternion.AngleAxis(-observation.theta * Mathf.Rad2Deg, Vector3.up) * direction;

        Debug.DrawRay(transform.position, direction * observation.r, color, duration);
    }
}
