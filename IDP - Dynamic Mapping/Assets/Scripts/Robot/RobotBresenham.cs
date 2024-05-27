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

    public Lidar lidar;
    public RobotController controller;

    public float maxConfidence = 0.99f;

    public int framesToConsiderStatic = 60;
    public int framesToConsiderDynamic = 20;

    private float HighStatic, LowStatic;
    private float HighDynamic, LowDynamic;
    

    private Texture2D texture;

    // Possible state for each cell of the grid:
    private const int FREE = 0;
    private const int OCCUPIED = 1;
    private const int UNKNOWN = 2;

    // Static and dynamic maps in log-odds form (the value of each cell can be computed
    // using the function getValue)
    private float[][] staticMap;
    private float[][] dynamicMap;

    private float maxLogOddValue;

    // Start is called before the first frame update
    void Start() {
        // Map width and height should be a multiple of 2:
        mapWidth &= ~1;
        mapHeight &= ~1;

        // Compute maxLogOdd values from maxConfidence:
        maxLogOddValue = Mathf.Log(maxConfidence / (1 - maxConfidence));

        // Compute the inverse sensor model values:
        float tmp = Mathf.Pow(0.95f / (1 - 0.95f), 1f / framesToConsiderStatic);
        HighStatic = tmp / (1 + tmp);
        LowStatic = 1 - HighStatic;

        tmp = Mathf.Pow(0.95f / (1 - 0.95f), 1f / framesToConsiderDynamic);
        HighDynamic = tmp / (1 + tmp);
        LowDynamic = 1 - HighDynamic;

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
        /*
        // Get the state of the robot from the controller:
        float robotX = controller.getRobotX();
        float robotY = controller.getRobotY();
        float robotAngle = controller.getRobotAngle();*/

        // Tempoary fix: Improve this
        float robotX = lidar.transform.position.x;
        float robotY = lidar.transform.position.z;
        float robotAngle = Mathf.Deg2Rad * (90 - lidar.transform.rotation.eulerAngles.y);

        // And update the static and dynamic maps using distances values from the LIDAR:
        updateMaps(robotX, robotY, robotAngle);
    }


    private void updateMaps(float robotX, float robotY, float robotAngle) {

        // Erase current map:
        for (int x = 0; x < mapWidth; x++)
            for (int y = 2 * mapHeight; y < 3 * mapHeight; y++)
                texture.SetPixel(x, y, Color.gray);

        // Get the observations returned from the LIDAR (-1 if no collision):
        Observation[] observations = lidar.GetObservations();

        // Position of the robot in the grids:
        int x0 = Mathf.FloorToInt(robotX / cellSize) + mapWidth / 2;
        int y0 = Mathf.FloorToInt(robotY / cellSize) + mapHeight / 2;

        // For each raycast, use Bresenham's algorithm to compute the intersection between the
        // raycast and the grids, and update the cells accordingly:
        foreach (Observation observation in observations) {
            if(observation == null) 
                continue;

            float angle = robotAngle + observation.theta;

            // Compute the position of the end of the ray, in world space:
            float xWorld, yWorld;
            if (observation.r < 0) {
                xWorld = robotX + Mathf.Cos(angle) * lidar.raycastDistance;
                yWorld = robotY + Mathf.Sin(angle) * lidar.raycastDistance;
            }
            else {
                xWorld = robotX + Mathf.Cos(angle) * observation.r;
                yWorld = robotY + Mathf.Sin(angle) * observation.r;
            }

            // Convert the world position into a cell position:
            int x1 = Mathf.FloorToInt(xWorld / cellSize) + mapWidth / 2;
            int y1 = Mathf.FloorToInt(yWorld / cellSize) + mapHeight / 2;

            // Update the cells touched by the raycast:
            bresenham(x0, y0, x1, y1, observation.r >= 0);
        }

        // Update the texture to reflect the changes on the maps:
        texture.Apply();
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
        float pSt = state == OCCUPIED && previousStaticValue > 0.1f ? HighStatic : LowStatic;
        float pDt = state == OCCUPIED && previousStaticValue <= 0.1f ? HighDynamic : LowDynamic;

        // Update the maps using equations (12) and (13):
        staticMap[x][y] += Mathf.Log(pSt / (1 - pSt));
        dynamicMap[x][y] += Mathf.Log(pDt / (1 - pDt));

        // Clamp values to avoid being too confident about the presence or absence
        // of an obstacle (this allows to take changes of the map into account):
        staticMap[x][y] = Mathf.Clamp(staticMap[x][y], -maxLogOddValue, maxLogOddValue);
        dynamicMap[x][y] = Mathf.Clamp(dynamicMap[x][y], -maxLogOddValue, maxLogOddValue);

        // Get the new values for the static and dynamic maps:
        float s = 1 - getMapValue(x, y, staticMap);
        float d = 1 - getMapValue(x, y, dynamicMap);
        texture.SetPixel(x, y + mapHeight, new Color(s, s, s));
        texture.SetPixel(x, y, new Color(d, d, d));
    }

    private float getMapValue(int x, int y, float[][] map)
    {
        // ln(value / (1 - value)) = map[x][y]
        // <=> value / (1 - value) = e^map[x][y]
        // <=> value = e^map[x][y] * (1 - value)
        // <=> value * (1 + e^map[x][y]) = e^map[x][y]

        float p = Mathf.Exp(map[x][y]);

        return p / (1 + p);
    }
}
