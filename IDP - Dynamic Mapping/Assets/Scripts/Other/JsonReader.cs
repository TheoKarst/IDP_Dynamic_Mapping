using System.IO;
using UnityEngine;

public class JsonReader : MonoBehaviour {

    [System.Serializable]
    public class FrameData {
        public int frame;
        public int sequence;
        public int step;
        public float timestamp;
        public Capture[] captures;
    }

    [System.Serializable]
    public class Capture {
        public string type;
        public string id;
        public string description;
        public float[] position;
        public float[] rotation;
        public float[] velocity;
        public float[] acceleration;
        public float[] globalPosition;
        public float[] globalRotation;
        public Annotation[] annotations;
    }

    [System.Serializable]
    public class Annotation {
        public string type;
        public string id;
        public string sensorId;
        public string description;
        public float[] ranges;
        public float[] angles;
        public string[] object_classes;
        public float[] intensities;
    }

    public class MyFrameData {
        public float timestamp;
        public Vector3 position;
        public Quaternion rotation;
        public float[] ranges;
        public float[] angles;

        public MyFrameData(float timestamp, Vector3 position, Quaternion rotation, 
            float[] ranges, float[] angles) {

            this.timestamp = timestamp;
            this.position = position;
            this.rotation = rotation;
            this.ranges = ranges;
            this.angles = angles;
        }
    }

    public bool restart = false;

    private MyFrameData currentFrame = null;
    private int currentStep = 0;
    private bool readingComplete = false;
    private float startTime;

    // Start is called before the first frame update
    void Start() {
        startTime = Time.time;
    }

    // Update is called once per frame
    void Update() {
        if(!readingComplete)
            LoadNextFrame("Assets/data/warehouse", Time.time - startTime);

        gameObject.transform.position = currentFrame.position;
        gameObject.transform.rotation = currentFrame.rotation;
    }

    public void OnDrawGizmos() {
        if(currentFrame != null) {

            for(int i = 0; i < currentFrame.angles.Length; i++) {
                float radius = currentFrame.ranges[i];
                float angle = currentFrame.angles[i];
                
                Vector3 direction = Quaternion.AngleAxis(angle, Vector3.up) * transform.forward;

                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, direction * radius);
            }
        }
    }

    public void OnValidate() {
        if(restart) {
            startTime = Time.time;
            currentStep = 0;
            currentFrame = null;
            readingComplete = false;
            restart = false;
        }
    }

    private void LoadNextFrame(string folder, float currentTime) {
        MyFrameData nextFrame = currentFrame;

        while(nextFrame == null || nextFrame.timestamp < currentTime) {
            string filename = folder + "/step" + currentStep + ".frame_data.json";
            if(!File.Exists(filename)) {
                readingComplete = true;
                return;
            }

            nextFrame = LoadData(filename);
            currentStep++;
        }

        if(nextFrame != null)
            currentFrame = nextFrame;
    }

    private MyFrameData LoadData(string filename) {
        string text = File.ReadAllText(filename);
        FrameData data = JsonUtility.FromJson<FrameData>(text);

        if (data == null || data.captures == null)
            return null;

        // Extract useful data from the JSON:
        float timestamp = data.timestamp;
        float[] position_data = data.captures[0].globalPosition;
        float[] rotation_data = data.captures[0].globalRotation;
        Annotation[] annotations = data.captures[0].annotations;

        if (position_data == null || rotation_data == null || annotations == null)
            return null;

        float[] ranges = annotations[0].ranges;
        float[] angles = annotations[0].angles;

        if(ranges == null || angles == null)
            return null;

        Vector3 position = ToVector3(position_data);
        Quaternion rotation = ToQuaternion(rotation_data);

        return new MyFrameData(timestamp, position, rotation, ranges, angles);
    }

    private string ToString(float[] data) {
        if (data == null)
            return "null";

        string result = "[";
        for(int i = 0; i < data.Length; i++) {
            result += data[i];
            if(i + 1 < data.Length) { result += "; "; }
        }
        return result + "]";
    }

    private Vector3 ToVector3(float[] data) {
        return new Vector3(data[0], data[1], data[2]);
    }

    private Quaternion ToQuaternion(float[] data) {
        return new Quaternion(data[0], data[1], data[2], data[3]);
    }
}
