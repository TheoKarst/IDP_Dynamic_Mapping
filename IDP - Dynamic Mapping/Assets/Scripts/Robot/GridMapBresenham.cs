using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using UnityEngine.UI;

public class GridMapBresenham : WorldModel
{
    // Script used to get the position of an observation in world space:
    private RobotManager manager;

    // Image and corresponding texture used to print the grid:
    private RawImage mapImage;
    private Texture2D texture;

    // Parameters of the grid map:
    private int pixelSize = 4;

    private float cellSize = 0.1f;
    private int mapWidth = 50;
    private int mapHeight = 50;

    private float maxConfidence = 0.99f;
    private int framesToConsiderStatic = 60;
    private int framesToConsiderDynamic = 20;

    // Values used to update the grid when new observations are made:
    private float HighStatic, LowStatic;
    private float HighDynamic, LowDynamic;

    // Possible state for each cell of the grid:
    private const int FREE = 0;
    private const int OCCUPIED = 1;
    private const int UNKNOWN = 2;

    // Static and dynamic maps in log-odds form (the value of each cell can be computed
    // using the function getValue)
    private float[][] staticMap;
    private float[][] dynamicMap;

    // Value in the grid corresponding to the maximum confidence:
    private float maxLogOddValue;

    public GridMapBresenham(RobotManager manager, RobotManager.GridMapParams gridMapParams) {
        this.manager = manager;

        this.mapImage = gridMapParams.mapImage;
        this.pixelSize = gridMapParams.pixelSize;

        this.cellSize = gridMapParams.cellSize;
        this.mapWidth = gridMapParams.mapWidth;
        this.mapHeight = gridMapParams.mapHeight;

        this.maxConfidence = gridMapParams.maxConfidence;
        this.framesToConsiderStatic = gridMapParams.framesToConsiderStatic;
        this.framesToConsiderDynamic = gridMapParams.framesToConsiderDynamic;

        // Create the grids and compute the required parameters to update the grid:
        SetupGrid();
    }

    private void SetupGrid() {
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

        texture = new Texture2D(mapWidth, 2 * mapHeight);
        texture.filterMode = FilterMode.Point;  // no smooth pixels

        mapImage.GetComponent<RectTransform>().sizeDelta = new Vector2(mapWidth * pixelSize, 2 * mapHeight * pixelSize);
        mapImage.texture = texture;
    }

    // Update the static and dynamic maps using the current sensor position and observations:
    public void UpdateMaps(Vector2 sensorPosition, float sensorAngle, ExtendedObservation[] observations) {

        // Position of the sensor in the grids:
        int x0 = Mathf.FloorToInt(sensorPosition.x / cellSize) + mapWidth / 2;
        int y0 = Mathf.FloorToInt(sensorPosition.y / cellSize) + mapHeight / 2;

        // For each raycast, use Bresenham's algorithm to compute the intersection between the
        // raycast and the grids, and update the cells accordingly:
        foreach (ExtendedObservation observation in observations) {
            float angle = sensorAngle + observation.theta;

            // Compute the position of the end of the ray, in world space:
            float xWorld = sensorPosition.x + Mathf.Cos(angle) * observation.r;
            float yWorld = sensorPosition.y + Mathf.Sin(angle) * observation.r;

            // Convert the world position into a cell position:
            int x1 = Mathf.FloorToInt(xWorld / cellSize) + mapWidth / 2;
            int y1 = Mathf.FloorToInt(yWorld / cellSize) + mapHeight / 2;

            // Update the cells touched by the raycast:
            Bresenham(x0, y0, x1, y1, observation.isValid);
        }
    }

    private void Bresenham(int x0, int y0, int x1, int y1, bool collision) {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (true) {
            if (x0 == x1 && y0 == y1) {

                // If there was a collision, the last cell touched by the raycast is occupied.
                // Else, it's free:
                UpdateCell(x0, y0, collision ? OCCUPIED : FREE);
                break;
            }

            // All the cells traversed by the raycast are free:
            UpdateCell(x0, y0, FREE);

            e2 = err;
            if (e2 > -dx) { err -= dy; x0 += sx; }
            if (e2 < dy) { err += dx; y0 += sy; }
        }
    }

    private void UpdateCell(int x, int y, int state) {
        if (state == UNKNOWN || x < 0 || x >= mapWidth || y < 0 || y >= mapHeight)
            return;

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

    public bool IsStatic(Observation observation) {
        // Compute the position of the observation, using the vehicle state estimate:
        Vector<double> position = manager.ComputeObservationPositionEstimate(observation);

        // Convert the world position into a cell position:
        int xCell = Mathf.FloorToInt((float) position[0] / cellSize) + mapWidth / 2;
        int yCell = Mathf.FloorToInt((float) position[1] / cellSize) + mapHeight / 2;

        // Check if there is a static cell around this cell:
        const int border = 5;
        for(int x = Mathf.Max(xCell - border, 0); x <= Mathf.Min(xCell + border, mapWidth-1); x++) {
            for (int y = Mathf.Max(yCell - border, 0); y <= Mathf.Min(yCell + border, mapHeight - 1); y++) {
                if (getMapValue(x, y, staticMap) >= 0.8f)
                    return true;
            }
        }

        return false;
    }

    public void DrawMap(bool showMap, bool updateTexture) {
        if(showMap && updateTexture) {
            Color[] pixels = new Color[2 * mapWidth * mapHeight];
            for (int x = 0; x < mapWidth; x++) {
                for (int y = 0; y < mapHeight; y++) {
                    // Gray color of the static cell:
                    float gray = 1 - getMapValue(x, y, staticMap);
                    pixels[mapWidth * y + x] = new Color(gray, gray, gray);

                    // Gray color of the dynamic cell:
                    gray = 1 - getMapValue(x, y, dynamicMap);
                    pixels[mapWidth * (mapHeight + y) + x] = new Color(gray, gray, gray);
                }
            }
            texture.SetPixels(pixels);
            texture.Apply();
        }

        mapImage.gameObject.SetActive(showMap);
    }
}
