using UnityEngine;

public class SineMotion : MonoBehaviour
{
    public float amplitudeX = 1;
    public float amplitudeRotationY = 45;
    public float omegaX = 1;
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
