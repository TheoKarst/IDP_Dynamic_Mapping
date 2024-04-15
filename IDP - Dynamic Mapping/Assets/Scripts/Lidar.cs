using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour
{
    public int raycastCount = 20;
    public float raycastDistance = 1;

    private float[] raycastDistances;


    // Start is called before the first frame update
    void Start()
    {
        raycastDistances = new float[raycastCount];
    }

    // Update is called once per frame
    void Update() {
        // Update the raycast distances each frame:
        Vector3 direction = transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < raycastCount; i++) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, raycastDistance)) {
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                raycastDistances[i] = hit.distance;
            }
            else {
                Debug.DrawRay(transform.position, direction * raycastDistance, Color.white);
                raycastDistances[i] = -1;
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
        }
    }

    public float[] getRaycastDistances() {
        return raycastDistances;
    }
}
