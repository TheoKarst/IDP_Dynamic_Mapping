using UnityEngine;

/// <summary>
/// Simple script used in Unity to randomly move the position of objects among different positions
/// </summary>
public class BoxManager : MonoBehaviour {

    [Tooltip("The box prefab to move")]
    public GameObject box;

    [Tooltip("The positions where a box can be placed")]
    public Transform[] positions;

    [Tooltip("The number of boxes to instantiate")]
    public int boxCount = 3;

    [Tooltip("Duration in seconds before a box is randomly placed at a different position")]
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
                int pos = GetFreePosition();
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
            int free = GetFreePosition();

            boxes[index].transform.SetParent(positions[free], false);
            freePosition[boxPosition[index]] = true;
            freePosition[free] = false;
            boxPosition[index] = free;
            lastTimeUpdate = Time.time;
        }
    }

    /// <summary>
    /// Returns a random free position among the possible positions
    /// </summary>
    private int GetFreePosition() {
        int position;
        do {
            position = Random.Range(0, positions.Length);
        }
        while (!freePosition[position]);

        return position;
    }
}
