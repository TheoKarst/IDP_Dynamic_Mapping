using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Class to build a dynamic map of an environment using two grids:
/// - A static grid for the static parts of the environment
/// - A dynamic grid for the dynamic parts of the environment
/// </summary>
public class GridMapBresenham {
    
    // Image and corresponding texture used to print the grid:
    private RawImage mapImage;
    private Texture2D texture;

    private float cellSize = 0.1f;
    private int mapWidth = 50;
    private int mapHeight = 50;

    private float maxConfidence = 0.99f;
    private int framesToConsiderStatic = 60;
    private int framesToConsiderDynamic = 5;

    // Values used to update the grid when new observations are made:
    private float HighStatic, LowStatic;
    private float HighDynamic, LowDynamic;

    // Static and dynamic maps in log-odds form (the value of each cell can be computed
    // using the function getValue)
    private float[][] staticMap;
    private float[][] dynamicMap;

    // Value in the grid corresponding to the maximum confidence:
    private float maxLogOddValue;

    /// <summary>
    /// Instantiates the mapping algorithm with the given parameters
    /// </summary>
    public GridMapBresenham(GridMapParams gridMapParams) {
        this.mapImage = gridMapParams.mapImage;

        this.cellSize = gridMapParams.cellSize;
        this.mapWidth = gridMapParams.mapWidth;
        this.mapHeight = gridMapParams.mapHeight;

        this.maxConfidence = gridMapParams.maxConfidence;
        this.framesToConsiderStatic = gridMapParams.framesToConsiderStatic;
        this.framesToConsiderDynamic = gridMapParams.framesToConsiderDynamic;

        // Create the grids and compute the required parameters to update the grid:
        SetupGrid();
    }

    /// <summary>
    /// Computes the parameters to update the grid, instantiates the static and dynamic maps
    /// and creates a texture to draw the map in the scene
    /// </summary>
    private void SetupGrid() {
        // Map width and height should be a multiple of 2:
        mapWidth &= ~1;
        mapHeight &= ~1;

        // Compute maxLogOdd values from maxConfidence:
        maxLogOddValue = Mathf.Log(maxConfidence / (1 - maxConfidence));

        // Compute the values for the inverse sensor model:
        float tmp = Mathf.Exp(maxLogOddValue / framesToConsiderStatic);
        HighStatic = tmp / (1 + tmp);
        LowStatic = 1 - HighStatic;

        tmp = Mathf.Exp(maxLogOddValue / framesToConsiderDynamic);
        HighDynamic = tmp / (1 + tmp);
        LowDynamic = 1 - HighDynamic;

        // Create the static and dynamic map, and initialize all values with 0:
        staticMap = new float[mapWidth][];
        dynamicMap = new float[mapWidth][];
        for (int i = 0; i < mapWidth; i++)
        {
            staticMap[i] = new float[mapHeight];
            dynamicMap[i] = new float[mapHeight];

            for (int j = 0; j < mapHeight; j++) {
                staticMap[i][j] = dynamicMap[i][j] = 0;
            }
        }

        // The texture is used to represent the static (on top) and the dynamic (on bottom) maps:
        texture = new Texture2D(mapWidth, 2 * mapHeight);
        texture.filterMode = FilterMode.Point;  // no smooth pixels
        mapImage.texture = texture;
    }

    /// <summary>
    /// Updates the static and dynamic maps using the current sensor position and observations
    /// </summary>
    /// <param name="worldSensorPose">World posie of the sensor which made the observations</param>
    /// <param name="observations">Observations of the sensor</param>
    public void UpdateMaps(Pose2D worldSensorPose, Observation[] observations) {

        // Position of the sensor in the grids:
        int x0 = Mathf.FloorToInt(worldSensorPose.x / cellSize) + mapWidth / 2;
        int y0 = Mathf.FloorToInt(worldSensorPose.y / cellSize) + mapHeight / 2;

        // For each raycast, use Bresenham's algorithm to compute the intersection between the
        // raycast and the grids, and update the cells accordingly:
        foreach (Observation observation in observations) {
            float angle = worldSensorPose.angle + observation.theta;

            // Compute the position of the end of the ray, in world space:
            float xWorld = worldSensorPose.x + Mathf.Cos(angle) * observation.r;
            float yWorld = worldSensorPose.y + Mathf.Sin(angle) * observation.r;

            // Convert the world position into a cell position:
            int x1 = Mathf.FloorToInt(xWorld / cellSize) + mapWidth / 2;
            int y1 = Mathf.FloorToInt(yWorld / cellSize) + mapHeight / 2;

            // Update the cells touched by the raycast:
            Bresenham(x0, y0, x1, y1, !observation.outOfRange);
        }
    }

    /// <summary>
    /// Uses Bresenhams algorithm to update the cells: all cells traversed by the line from (x0, y0)
    /// to (x1, y1) are considered free. The cell (x1, y1) is considered occupied if collision=True
    /// and free otherwise
    /// </summary>
    private void Bresenham(int x0, int y0, int x1, int y1, bool collision) {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (true) {
            if (x0 == x1 && y0 == y1) {

                // If there was a collision, the last cell touched by the raycast is occupied.
                // Else, it's free:
                UpdateCell(x0, y0, collision);
                break;
            }

            // All the cells traversed by the raycast are free:
            UpdateCell(x0, y0, false);

            e2 = err;
            if (e2 > -dx) { err -= dy; x0 += sx; }
            if (e2 < dy) { err += dx; y0 += sy; }
        }
    }

    /// <summary>
    /// Updates the given cell, knowing how the cell is observed
    /// </summary>
    /// <param name="occupied">Should be true if the cell was observed occupied and false if it was observed
    /// free. If the cell is not observed, this function shouldn't be called because the state of the cell
    /// should stay the same</param>
    private void UpdateCell(int x, int y, bool occupied) {
        if (x < 0 || x >= mapWidth || y < 0 || y >= mapHeight)
            return;

        float previousStaticValue = LogOddsToProba(staticMap[x][y]);

        // Update the static and dynamic maps according to the inverse observation model
        // of the static and dynamic maps:
        float pSt = occupied && previousStaticValue > 0.1f ? HighStatic : LowStatic;
        float pDt = occupied && previousStaticValue <= 0.1f ? HighDynamic : LowDynamic;

        // Update the maps using equations (12) and (13):
        staticMap[x][y] += ProbaToLogOdds(pSt);
        dynamicMap[x][y] += ProbaToLogOdds(pDt);

        // Clamp values to avoid being too confident about the presence or absence
        // of an obstacle (this allows to take changes of the map into account):
        staticMap[x][y] = Mathf.Clamp(staticMap[x][y], -maxLogOddValue, maxLogOddValue);
        dynamicMap[x][y] = Mathf.Clamp(dynamicMap[x][y], -maxLogOddValue, maxLogOddValue);
    }

    /// <summary>Returns the log-odds form representation of a probability</summary>
    private float ProbaToLogOdds(float p) {
        return Mathf.Log(p / (1 - p));
    }

    /// <summary>Returns the probability represented by the given log-odds form</summary>
    private float LogOddsToProba(float l) {
        float e = Mathf.Exp(l);

        return e / (1 + e);
    }

    /// <summary>
    /// Returns if the given position corresponds to a static object or not
    /// </summary>
    public bool IsStatic(Vector2 worldPosition) {

        // Convert the world position into a cell position:
        int xCell = Mathf.FloorToInt(worldPosition.x / cellSize) + mapWidth / 2;
        int yCell = Mathf.FloorToInt(worldPosition.y / cellSize) + mapHeight / 2;

        // Instead of checking just one cell, check if at least one neighbouring cell is static.
        // This method is a bit slower but more robust to errors in the estimate of the observation:
        const int border = 5;
        for(int x = Mathf.Max(xCell - border, 0); x <= Mathf.Min(xCell + border, mapWidth-1); x++) {
            for (int y = Mathf.Max(yCell - border, 0); y <= Mathf.Min(yCell + border, mapHeight - 1); y++) {
                if (LogOddsToProba(staticMap[x][y]) >= 0.8f)
                    return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Draws the map in the scene
    /// </summary>
    /// <param name="showMap">If we need to show the map in the scene</param>
    /// <param name="updateTexture">If we need to update the texture representing the map</param>
    public void DrawMap(bool showMap, bool updateTexture) {
        if(showMap && updateTexture) {
            // Should contain the pixels row by row, starting at the bottom left of the texture:
            Color[] pixels = new Color[2 * mapWidth * mapHeight];

            for (int x = 0; x < mapWidth; x++) {
                for (int y = 0; y < mapHeight; y++) {
                    // Gray color of the static cell. If the occupancy probability is 0,
                    // the cell should be white:
                    float gray = 1 - LogOddsToProba(staticMap[x][y]);
                    pixels[mapWidth * (mapHeight + y) + x] = new Color(gray, gray, gray);

                    // Gray color of the dynamic cell. If the occupancy probability is 0,
                    // the cell should be white:
                    gray = 1 - LogOddsToProba(dynamicMap[x][y]);
                    pixels[mapWidth * y + x] = new Color(gray, gray, gray);
                }
            }

            texture.SetPixels(pixels);
            texture.Apply();
        }

        mapImage.gameObject.SetActive(showMap);
    }
}
