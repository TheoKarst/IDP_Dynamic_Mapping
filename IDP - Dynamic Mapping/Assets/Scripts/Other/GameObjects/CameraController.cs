using UnityEngine;

/// <summary>
/// Simple script used in Unity to track an object with the camera
/// </summary>
public class CameraController : MonoBehaviour {
    
    [Tooltip("The object this camera should follow")]
    public GameObject target;

    [Tooltip("How much the camera should zoom in/zoom out when pressing W and S keys")]
    public float zoomSpeed = 1;

    public void LateUpdate() {
        // Use the (x, z) position of the target to set the position of the camera.
        // The y position of the camera should stay the same:
        Vector3 position = new Vector3(target.transform.position.x, transform.position.y, target.transform.position.z);
        
        // Then we can update the y position of the camera using the keyboard:
        if(Input.GetKey(KeyCode.W)) {
            position.y -= zoomSpeed * Time.deltaTime;
        }
        else if(Input.GetKey(KeyCode.S)) {
            position.y += zoomSpeed * Time.deltaTime;
        }

        gameObject.transform.position = position;
    }
}
