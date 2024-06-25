using UnityEngine;

public class MovingObject : MonoBehaviour
{
    public Transform[] transforms;
    public float speed = 1f;
    public float waitDuration = 1f;

    private Vector3 lastPosition;
    private Quaternion lastRotation;

    private Vector3 targetPosition;
    private Quaternion targetRotation;

    private float startTime;
    private float duration;
    private int targetIndex = 0;

    // Start is called before the first frame update
    void Start()
    {
        lastPosition = gameObject.transform.position;
        lastRotation = gameObject.transform.rotation;
        targetPosition = transforms[targetIndex].position;
        targetRotation = transforms[targetIndex].rotation;

        startTime = Time.time;
        duration = (targetPosition - lastPosition).magnitude / speed;
    }

    // Update is called once per frame
    void Update()
    {
        float t = Time.time - startTime;

        if(t <= duration) {
            gameObject.transform.position = Vector3.Lerp(lastPosition, targetPosition, t / duration);
            gameObject.transform.rotation = Quaternion.Lerp(lastRotation, targetRotation, t / duration);
        }
        else if(t > duration + waitDuration) {

            // Go to next target:
            targetIndex = (targetIndex + 1) % transforms.Length;
            lastPosition = targetPosition;
            lastRotation = targetRotation;

            targetPosition = transforms[targetIndex].position;
            targetRotation = transforms[targetIndex].rotation;

            startTime = Time.time;
            duration = (targetPosition - lastPosition).magnitude / speed;
        }
        else {
            gameObject.transform.position = targetPosition;
            gameObject.transform.rotation = targetRotation;
        }
    }
}
