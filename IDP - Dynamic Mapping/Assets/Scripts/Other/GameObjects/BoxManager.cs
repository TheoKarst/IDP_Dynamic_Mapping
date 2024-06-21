using UnityEngine;

public class BoxManager : MonoBehaviour
{
    public GameObject box;
    public Transform[] positions;
    public int boxCount = 3;
    public float durationBetweenUpdates = 2;

    private GameObject[] boxes;
    private int[] boxPosition;
    private float lastTimeUpdate;
    private bool[] freePosition;

    // Start is called before the first frame update
    void Start() {
        // There cannot be more boxes than the number of positions:
        boxCount = Mathf.Min(positions.Length, boxCount);

        // Instantiate boxes:
        boxes = new GameObject[boxCount];
        boxPosition = new int[boxCount];
        freePosition = new bool[positions.Length];
        for(int i = 0; i < positions.Length; i++)
            freePosition[i] = true;

        if(boxCount == positions.Length) {
            for (int i = 0; i < boxCount; i++) {
                boxes[i] = Instantiate(box, positions[i]);
                boxPosition[i] = i;
                freePosition[i] = false;
            }
        }
        else {
            for(int i = 0; i < boxCount; i++) {
                int pos = getFreePosition();
                boxes[i] = Instantiate(box, positions[pos]);
                boxPosition[i] = pos;
                freePosition[pos] = false;
            }
        }

        lastTimeUpdate = Time.time;
    }

    // Update is called once per frame
    void Update() {
        if (boxCount == positions.Length)
            return;

        if(Time.time  - lastTimeUpdate >= durationBetweenUpdates) {
            // Select a box, and move it to a different free position:
            int index = Random.Range(0, boxCount);
            int free = getFreePosition();

            boxes[index].transform.SetParent(positions[free], false);
            freePosition[boxPosition[index]] = true;
            freePosition[free] = false;
            boxPosition[index] = free;
            lastTimeUpdate = Time.time;
        }
    }

    private int getFreePosition() {
        int position;
        do {
            position = Random.Range(0, positions.Length);
        }
        while (!freePosition[position]);

        return position;
    }
}
