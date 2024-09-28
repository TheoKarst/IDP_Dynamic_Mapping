using UnityEngine;

/// <summary>
/// Simple script to move objects in Unity scene
/// </summary>

public class MovingObject : MonoBehaviour {

    [Tooltip("The list of transforms where this object should go")]
    public Transform[] transforms;

    [Tooltip("If true, the object will always move at the same movingSpeed. Otherwise," +
        "the movingSpeed is ignored and the moveDuration is used.")]
    public bool constantSpeed = false;

    [Tooltip("Speed of the object")]
    public float movingSpeed = 1;

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

    private float currentMoveDuration;

    // Start is called before the first frame update
    void Start()
    {
        lastPosition = gameObject.transform.position;
        lastRotation = gameObject.transform.rotation;
        targetPosition = transforms[targetIndex].position;
        targetRotation = transforms[targetIndex].rotation;

        currentMoveDuration = ComputeMoveDuration();

        startTime = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
        float t = Time.time - startTime;

        if(t < currentMoveDuration) {
            gameObject.transform.position = Vector3.Lerp(lastPosition, targetPosition, t / currentMoveDuration);
            gameObject.transform.rotation = Quaternion.Lerp(lastRotation, targetRotation, t / currentMoveDuration);
        }
        else if(t > currentMoveDuration + waitDuration) {

            // Go to next target:
            targetIndex = (targetIndex + 1) % transforms.Length;
            lastPosition = targetPosition;
            lastRotation = targetRotation;

            targetPosition = transforms[targetIndex].position;
            targetRotation = transforms[targetIndex].rotation;

            currentMoveDuration = ComputeMoveDuration();

            startTime = Time.time;
        }
        else {
            gameObject.transform.position = targetPosition;
            gameObject.transform.rotation = targetRotation;
        }
    }

    private float ComputeMoveDuration() {
        if (constantSpeed)
            return (lastPosition - targetPosition).magnitude / movingSpeed;
        
        else
            return moveDuration;
    }
}
