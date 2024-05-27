using System.Runtime.ConstrainedExecution;
using UnityEngine;
using UnityEngine.UI;

public class Robot : MonoBehaviour
{
    public RawImage image;
    public int pixelSize = 4;

    [Tooltip("Size of each cell")]
    public float cellSize = 0.1f;
    public int mapWidth = 50;
    public int mapHeight = 50;

    public int raycastCount = 20;
    public float raycastDistance = 1;

    public float clampOdds = 1.5f;

    private Texture2D texture;

    // Possible state for each cell of the grid:
    private const int FREE = 0;
    private const int OCCUPIED = 1;
    private const int UNKNOWN = 2;

    // Static and dynamic maps in log-odds form (the value of each cell can be computed
    // using the function getValue)
    private float[][] staticMap;
    private float[][] dynamicMap;

    private const float High = 0.55f;    // High probability (inverse sensor model)
    private const float Low = 0.45f;     // Low probability (inverse sensor model)

    // Start is called before the first frame update
    void Start()
    {
        // Map width and height should be a multiple of 2:
        mapWidth &= ~1;
        mapHeight &= ~1;

        // Create the static and dynamic map, and initialize all values with 0:
        staticMap = new float[mapWidth][];
        dynamicMap = new float[mapWidth][];
        for(int i = 0; i < mapWidth; i++) {
            staticMap[i] = new float[mapHeight];
            dynamicMap[i] = new float[mapHeight];

            for(int j = 0; j < mapHeight; j++) {
                staticMap[i][j] = dynamicMap[i][j] = 0;
            }
        }

        texture = new Texture2D(mapWidth, 3 * mapHeight);
        texture.filterMode = FilterMode.Point;  // no smooth pixels

        image.GetComponent<RectTransform>().sizeDelta = new Vector2(mapWidth * pixelSize, 3 * mapHeight * pixelSize);
        image.texture = texture;
    }

    // Update is called once per frame
    void Update() {
        float robotX = gameObject.transform.position.x;
        float robotY = gameObject.transform.position.z;
        float robotAngle = Mathf.Deg2Rad * (90 - gameObject.transform.eulerAngles.y);

        float[] distances = raycastDistances();

        for(int x = 0; x < mapWidth; x++) {
            for(int y = 0; y < mapHeight; y++) {
                int cellState = getCellState(x, y, robotX, robotY, robotAngle, distances);

                // Update the texture to reflect the current state of the cells:
                if (cellState == FREE)
                    texture.SetPixel(x, y, Color.white);
                else if (cellState == OCCUPIED)
                    texture.SetPixel(x, y, Color.black);
                else {
                    texture.SetPixel(x, y, Color.gray);

                    // If the cell state is unknown, no need to update the static or dynamic maps:
                    continue;
                }

                float previousStaticValue = getValue(x, y, staticMap);

                // Update the static and dynamic maps according to Tables 1 and 2
                // (inverse observation model for the static and dynamic maps):
                float pSt = cellState == OCCUPIED && previousStaticValue > 0.1f ? High : Low;
                float pDt = cellState == OCCUPIED && previousStaticValue <= 0.1f ? High : Low;

                // Update the maps using equations (12) and (13):
                staticMap[x][y] += Mathf.Log10(pSt / (1 - pSt));
                dynamicMap[x][y] += Mathf.Log10(pDt / (1 - pDt));

                // Clamp values to avoid being too confident about the presence or absence
                // of an obstacle (this allows to take changes of the map into account):
                staticMap[x][y] = Mathf.Clamp(staticMap[x][y], -clampOdds, clampOdds);
                dynamicMap[x][y] = Mathf.Clamp(dynamicMap[x][y], -clampOdds, clampOdds);

                // Get the new values for the static and dynamic maps:
                float s = 1 - getValue(x, y, staticMap);
                float d = 1 - getValue(x, y, dynamicMap);
                texture.SetPixel(x, y + mapHeight, new Color(s, s, s));
                texture.SetPixel(x, y + 2 * mapHeight, new Color(d, d, d));
            }
        }
        texture.Apply();
    }

    private float[] raycastDistances() {
        float[] result = new float[raycastCount];

        Vector3 direction = transform.TransformDirection(Vector3.forward);

        for(int i = 0; i < raycastCount; i++) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, raycastDistance))
            {
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                result[i] = hit.distance;
            }
            else
            {
                Debug.DrawRay(transform.position, direction * raycastDistance, Color.white);
                result[i] = -1;
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
        }

        return result;
    }

    // robotAngle: Angle in radians between the robot forward and the right axis
    private int getCellState(int cellX, int cellY, 
        float robotX, float robotY, float robotAngle, float[] raycastDistances)
    {
        // Distance between the center of the cell and the robot center, along X and Y axes:
        float dX = (0.5f + cellX - mapWidth/2) * cellSize - robotX;
        float dY = (0.5f + cellY - mapHeight/2) * cellSize - robotY;

        // Euclidean distance between the robot center and the cell:
        float r = Mathf.Sqrt(dX*dX + dY*dY);

        // If the cell is out of the range of the sensor, its state is unknown:
        if (r > raycastDistance)
            return UNKNOWN;

        // Else, we search the nearest raycast to the cell, and compute the angle
        // between the cell and this raycast:
        const float TWO_PI = 2 * Mathf.PI;

        // Angle between two raycasts:
        float delta = TWO_PI / raycastCount;

        // Angle between the cell and the robot's forward
        float alpha = Mathf.Atan2(dY, dX) - robotAngle;
        while (alpha >= TWO_PI) alpha -= TWO_PI;
        while (alpha < 0) alpha += TWO_PI;

        // Index of the nearest raycast:
        int index = Mathf.FloorToInt(0.5f + alpha / delta);
        if (index >= raycastCount) index -= raycastCount;

        // Update alpha to be the angle between the cell and the nearest raycast:
        alpha -= index * delta;

        // If the angle between the cell and the nearest raycast is too high, there is no
        // intersection between the ray and the cell, so the cell's state is unknown:
        if (Mathf.Abs(alpha) > cellSize / (2 * r))
            return UNKNOWN;

        // Else if the nearest raycast collided nothing, then the cell is free:
        if (raycastDistances[index] == -1)
            return FREE;

        // Else if the raycast collided this cell, the cell is occupied:
        if (Mathf.Abs(raycastDistances[index] - r) < cellSize/2)
            return OCCUPIED;

        // Else, if the cell is before the collision point, it's free, and if it's after, the cell
        // state is unknown:
        return r < raycastDistances[index] ? FREE : UNKNOWN;
    }

    private float getValue(int x, int y, float[][] map)
    {
        // log(value / (1 - value)) = map[x][y]
        // <=> value / (1 - value) = 10^map[x][y]
        // <=> value = 10^map[x][y] * (1 - value)
        // <=> value * (1 + 10^map[x][y]) = 10^map[x][y]

        float p = Mathf.Pow(10, map[x][y]);

        return p / (1 + p);
    }
}
