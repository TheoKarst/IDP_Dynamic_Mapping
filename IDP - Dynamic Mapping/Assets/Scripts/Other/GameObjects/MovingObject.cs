using UnityEngine;

/// <summary>
/// Simple script to move objects in Unity scene
/// </summary>
public class MovingObject : MonoBehaviour {

    [Tooltip("The list of transforme where this object should go")]
    public Transform[] transforms;

    [Tooltip("Duration to move from one transform to the next")]
    public float moveDuration = 1f;

    [Tooltip("Duration to wait at each transform")]
    public float waitDuration = 1f;

    private Vector3 lastPosition;
    private Quaternion lastRotation;

    private Vector3 targetPosition;
    private Quaternion targetRotation;

    private float startTime;
    private int targetIndex = 0;

    // Start is called before the first frame update
    void Start()
    {
        lastPosition = gameObject.transform.position;
        lastRotation = gameObject.transform.rotation;
        targetPosition = transforms[targetIndex].position;
        targetRotation = transforms[targetIndex].rotation;

        startTime = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
        float t = Time.time - startTime;

        if(t <= moveDuration) {
            gameObject.transform.position = Vector3.Lerp(lastPosition, targetPosition, t / moveDuration);
            gameObject.transform.rotation = Quaternion.Lerp(lastRotation, targetRotation, t / moveDuration);
        }
        else if(t > moveDuration + waitDuration) {

            // Go to next target:
            targetIndex = (targetIndex + 1) % transforms.Length;
            lastPosition = targetPosition;
            lastRotation = targetRotation;

            targetPosition = transforms[targetIndex].position;
            targetRotation = transforms[targetIndex].rotation;

            startTime = Time.time;
        }
        else {
            gameObject.transform.position = targetPosition;
            gameObject.transform.rotation = targetRotation;
        }
    }
}
