using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RobotBresenham : MonoBehaviour
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
        for (int i = 0; i < mapWidth; i++)
        {
            staticMap[i] = new float[mapHeight];
            dynamicMap[i] = new float[mapHeight];

            for (int j = 0; j < mapHeight; j++)
            {
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

        updateMaps(robotX, robotY, robotAngle);
    }


    private void updateMaps(float robotX, float robotY, float robotAngle) {

        // Erase current map:
        for (int x = 0; x < mapWidth; x++)
            for (int y = 2 * mapHeight; y < 3 * mapHeight; y++)
                texture.SetPixel(x, y, Color.gray);

        // Compute the distances returned from the LIDAR (-1 if no collision):
        float[] distances = raycastDistances();

        // Compute the angle in radians between two raycasts:
        float delta = 2 * Mathf.PI / raycastCount;

        // Position of the robot in the grids:
        int x0 = Mathf.FloorToInt(robotX / cellSize) + mapWidth / 2;
        int y0 = Mathf.FloorToInt(robotY / cellSize) + mapHeight / 2;

        // For each raycast, use Bresenham's algorithm to compute the intersection between the
        // raycast and the grids, and update the cells accordingly:
        for (int i = 0; i < distances.Length; i++) {
            float angle = robotAngle + i * delta;

            // Compute the position of the end of the ray, in world space:
            float xWorld, yWorld;
            if (distances[i] < 0) {
                xWorld = robotX + Mathf.Cos(angle) * raycastDistance;
                yWorld = robotY + Mathf.Sin(angle) * raycastDistance;
            }
            else {
                xWorld = robotX + Mathf.Cos(angle) * distances[i];
                yWorld = robotY + Mathf.Sin(angle) * distances[i];
            }

            // Convert the world position into a cell position:
            int x1 = Mathf.FloorToInt(xWorld / cellSize) + mapWidth / 2;
            int y1 = Mathf.FloorToInt(yWorld / cellSize) + mapHeight / 2;

            // Update the cells touched by the raycast:
            bresenham(x0, y0, x1, y1, distances[i] >= 0);
        }

        // Update the texture to reflect the changes on the maps:
        texture.Apply();
    }

    private float[] raycastDistances() {
        float[] result = new float[raycastCount];

        Vector3 direction = transform.TransformDirection(Vector3.forward);

        for (int i = 0; i < raycastCount; i++) {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, raycastDistance)) {
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                result[i] = hit.distance;
            }
            else {
                Debug.DrawRay(transform.position, direction * raycastDistance, Color.white);
                result[i] = -1;
            }

            // Rotate the direction of the raycast counterclockwise:
            direction = Quaternion.AngleAxis(-360f / raycastCount, Vector3.up) * direction;
        }

        return result;
    }

    private void bresenham(int x0, int y0, int x1, int y1, bool collision) {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (true) {
            if (x0 == x1 && y0 == y1) {

                // If there was a collision, the last cell touched by the raycast is occupied.
                // Else, it's free:
                updateCell(x0, y0, collision ? OCCUPIED : FREE);
                break;
            }

            // All the cells traversed by the raycast are free:
            updateCell(x0, y0, FREE);

            e2 = err;
            if (e2 > -dx) { err -= dy; x0 += sx; }
            if (e2 < dy) { err += dx; y0 += sy; }
        }
    }

    private void updateCell(int x, int y, int state) {
        if (state == UNKNOWN || x < 0 || x >= mapWidth || y < 0 || y >= mapHeight)
            return;

        // Update the current cell color:
        texture.SetPixel(x, y + 2 * mapHeight, state == FREE ? Color.white : Color.black);

        float previousStaticValue = getMapValue(x, y, staticMap);

        // Update the static and dynamic maps according to Tables 1 and 2
        // (inverse observation model for the static and dynamic maps):
        float pSt = staticInverseModel(previousStaticValue, state == FREE);
        float pDt = dynamicInverseModel(previousStaticValue, state == FREE);

        // Update the maps using equations (12) and (13):
        staticMap[x][y] += Mathf.Log10(pSt / (1 - pSt));
        dynamicMap[x][y] += Mathf.Log10(pDt / (1 - pDt));

        // Clamp values to avoid being too confident about the presence or absence
        // of an obstacle (this allows to take changes of the map into account):
        staticMap[x][y] = Mathf.Clamp(staticMap[x][y], -clampOdds, clampOdds);
        dynamicMap[x][y] = Mathf.Clamp(dynamicMap[x][y], -clampOdds, clampOdds);

        // Get the new values for the static and dynamic maps:
        float s = 1 - getMapValue(x, y, staticMap);
        float d = 1 - getMapValue(x, y, dynamicMap);
        texture.SetPixel(x, y + mapHeight, new Color(s, s, s));
        texture.SetPixel(x, y, new Color(d, d, d));
    }

    // Implementation of Table 1:
    private float staticInverseModel(float previousStatic, bool free) {
        if(free) {
            if (previousStatic < 0.1f) return 0.3f;     // FREE => FREE
            if (previousStatic < 0.9f) return 0.4f;     // UNKNOWN => FREE
            return 0.45f;                               // OCCUPIED => FREE
        }
        else {
            if (previousStatic < 0.1f) return 0.55f;    // FREE => OCCUPIED
            if (previousStatic < 0.9f) return 0.6f;     // UNKNOWN => OCCUPIED
            return 0.7f;                                // OCCUPIED => OCCUPIED
        }
    }

    // Implementation of Table 2:
    private float dynamicInverseModel(float previousStatic, bool free) {
        if (free) {
            if (previousStatic < 0.1f) return 0.3f;     // FREE => FREE
            if (previousStatic < 0.9f) return 0.4f;     // UNKNOWN => FREE
            return 0.4f;                                // OCCUPIED => FREE
        }
        else {
            if (previousStatic < 0.1f) return 0.9f;     // FREE => OCCUPIED
            if (previousStatic < 0.9f) return 0.6f;     // UNKNOWN => OCCUPIED
            return 0.3f;                                // OCCUPIED => OCCUPIED
        }
    }

    private float getMapValue(int x, int y, float[][] map)
    {
        // log(value / (1 - value)) = map[x][y]
        // <=> value / (1 - value) = 10^map[x][y]
        // <=> value = 10^map[x][y] * (1 - value)
        // <=> value * (1 + 10^map[x][y]) = 10^map[x][y]

        float p = Mathf.Pow(10, map[x][y]);

        return p / (1 + p);
    }
}
