using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingObject : MonoBehaviour
{
    public Transform[] transforms;
    public float speed = 1f;
    public float waitDuration = 1f;

    private Vector3 lastPosition;
    private Vector3 targetPosition;
    private float startTime;
    private float duration;
    private int targetIndex = 0;

    // Start is called before the first frame update
    void Start()
    {
        lastPosition = gameObject.transform.position;
        targetPosition = transforms[targetIndex].position;

        startTime = Time.time;
        duration = (targetPosition - lastPosition).magnitude / speed;
    }

    // Update is called once per frame
    void Update()
    {
        float t = Time.time - startTime;

        if(t <= duration) {
            gameObject.transform.position = Vector3.Lerp(lastPosition, targetPosition, t / duration);
        }
        else if(t > duration + waitDuration) {

            // Go to next target:
            targetIndex = (targetIndex + 1) % transforms.Length;
            lastPosition = targetPosition;
            targetPosition = transforms[targetIndex].position;
            startTime = Time.time;
            duration = (targetPosition - lastPosition).magnitude / speed;
        }
        else {
            gameObject.transform.position = targetPosition;
        }
    }
}
