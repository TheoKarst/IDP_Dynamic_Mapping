using UnityEngine;

/// <summary>
/// Simple script to move an object with a sine motion
/// </summary>
public class SineMotion : MonoBehaviour {

    [Tooltip("Amplitude of the motion along the X-axis")]
    public float amplitudeX = 1;

    [Tooltip("Amplitude of the rotation along the Y-axis")]
    public float amplitudeRotationY = 45;

    [Tooltip("Speed of the motion along the X-axis")]
    public float omegaX = 1;

    [Tooltip("Speed of the motion along the Y-axis")]
    public float omegaY = 1;

    private float startX, startAngleY;

    // Start is called before the first frame update
    void Start()
    {
        startX = transform.position.x;
        startAngleY = transform.rotation.eulerAngles.y;
    }

    // Update is called once per frame
    void Update()
    {
        float t = Time.realtimeSinceStartup;

        Vector3 position = new Vector3(
            startX + amplitudeX * Mathf.Cos(omegaX * t),
            transform.position.y,
            transform.position.z);

        Vector3 rotation = new Vector3(
            transform.rotation.eulerAngles.x,
            startAngleY + amplitudeRotationY * Mathf.Sin(omegaY * t),
            transform.rotation.eulerAngles.z);

        transform.position = position;
        transform.rotation = Quaternion.Euler(rotation);
    }
}
