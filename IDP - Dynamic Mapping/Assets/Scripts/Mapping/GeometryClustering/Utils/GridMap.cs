// Class used to project lines into a grid, in order to compute faster
// line matching:
using System;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Assertions;

public class GridMap {
    private int width;
    private int height;

    // Position of the top left corner of the grid:
    private float cornerX, cornerY;

    private float cellSize;

    private List<DynamicLine>[] cells;

    public GridMap(int width, int height, float cellSize) 
        : this(width, height, 0, 0, cellSize) {}

    public GridMap(int width, int height, float centerX, float centerY, float cellSize) {
        this.width = width;
        this.height = height;
        
        this.cornerX = centerX - width * cellSize / 2;
        this.cornerY = centerY + height * cellSize / 2;

        this.cellSize = cellSize;

        cells = new List<DynamicLine>[width * height];
    }

    public void RegisterLine(DynamicLine line) {
        // Compute the position of the endpoints of the line in the grid:
        float x0 = (line.beginPoint.x - cornerX) / cellSize;
        float y0 = (cornerY - line.beginPoint.y) / cellSize;
        float x1 = (line.endPoint.x - cornerX) / cellSize;
        float y1 = (cornerY - line.endPoint.y) / cellSize;

        if(!BelongsToGrid(x0, y0) || !BelongsToGrid(x1, y1)) {
            Debug.LogError("The given line is outside of the bounds of the grid !");
            return;
        }

        List<(int, int)> cells = FindContactCells(x0, y0, x1, y1);
        foreach((int x, int y) cell in cells)
            RegisterLine(line, cell.x, cell.y);
    }

    public HashSet<DynamicLine> FindNeighbors(DynamicLine line) {
        HashSet<DynamicLine> neighbors = new HashSet<DynamicLine>();

        // Compute the position of the endpoints of the line in the grid:
        float x0 = (line.beginPoint.x - cornerX) / cellSize;
        float y0 = (cornerY - line.beginPoint.y) / cellSize;
        float x1 = (line.endPoint.x - cornerX) / cellSize;
        float y1 = (cornerY - line.endPoint.y) / cellSize;

        Assert.IsTrue(BelongsToGrid(x0, y0) && BelongsToGrid(x1, y1),
            "The given line is outside of the bounds of the grid !");

        List<(int, int)> cells = FindContactCellsMargin(x0, y0, x1, y1);
        foreach ((int x, int y) cell in cells) {
            List<DynamicLine> cellLines = this[cell.x, cell.y];

            if(cellLines != null)
                neighbors.AddRange(cellLines);
        }
        
        return neighbors;
    }

    public List<DynamicLine> FindNeighbors(Circle circle) {
        // Compute the position of the center of the circle in the grid:

        int x = (int) ((circle.position.x - cornerX) / cellSize);
        int y = (int) ((cornerY - circle.position.y) / cellSize);

        Assert.IsTrue(BelongsToGrid(x, y), "The given circle is outside of the grid !");

        return this[x, y];
    }

    // Find all the cells that are colliding the line with the given endpoints, expressed
    // in the grid coordinate system, using the voxel traversal algorithm described in the
    // following paper: https://www.cse.chalmers.se/edu/year/2013/course/TDA361/grid.pdf
    private List<(int, int)> FindContactCells(float x0, float y0, float x1, float y1) {
        List<(int, int)> result = new List<(int, int)>();

        int x = (int)x0, y = (int)y0;
        int xEnd = (int)x1, yEnd = (int)y1;

        int stepX = x0 <= x1 ? 1 : -1;
        int stepY = y0 <= y1 ? 1 : -1;

        if (x == xEnd) {
            result.Add((x, y));
            while (y != yEnd) {
                y += stepY;
                result.Add((x, y));
            }
            return result;
        }

        if (y == yEnd) {
            result.Add((x, y));
            while (x != xEnd) {
                x += stepX;
                result.Add((x, y));
            }
            return result;
        }

        float tMaxX = (x0 < x1 ? x + stepX - x0 : x - x0) / (x1 - x0);
        float tMaxY = (y0 < y1 ? y + stepY - y0 : y - y0) / (y1 - y0);

        float slopeX = 1 / Mathf.Abs(x1 - x0);
        float slopeY = 1 / Mathf.Abs(y1 - y0);

        int stepsCount = Mathf.Abs(xEnd - x) + Mathf.Abs(yEnd - y);
        for (int i = 0; i <= stepsCount; i++) {
            result.Add((x, y));

            if (tMaxX < tMaxY) {
                tMaxX += slopeX;
                x += stepX;
            }
            else {
                tMaxY += slopeY;
                y += stepY;
            }
        }

        return result;
    }

    // Use a modified version of Xiaolin Wu's algorithm, to get all the cells colliding the line
    // with the given endpoints, returning all the points explored by the algorithm, no matter
    // the brightness:
    private List<(int, int)> FindContactCellsMargin(float x0, float y0, float x1, float y1) {
        List<(int, int)> result = new List<(int, int)>();

        bool steep = Mathf.Abs(y1 - y0) > Mathf.Abs(x1 - x0);

        if (steep) {
            (x0, y0) = (y0, x0);
            (x1, y1) = (y1, x1);
        }
        if (x0 > x1) {
            (x0, x1) = (x1, x0);
            (y0, y1) = (y1, y0);
        }

        float dx = x1 - x0, dy = y1 - y0;
        float gradient = dx == 0 ? 1 : dy / dx;

        // Handle first endpoint:
        int xend = (int) x0;
        float yend = y0 + gradient * (xend - x0) + (gradient - 1) / 2;
        int xpxl1 = xend;  // This will be used in the main loop
        int ypxl1 = (int) yend;
        if (steep) {
            result.Add((ypxl1, xpxl1));
            result.Add((ypxl1 + 1, xpxl1));
        }
        else {
            result.Add((xpxl1, ypxl1));
            result.Add((xpxl1, ypxl1 + 1));
        }

        // First y-intersection for the main loop:
        float intery = yend + gradient;

        // Handle second endpoint:
        xend = (int) x1;
        yend = y1 + gradient * (xend - x1) + (gradient - 1) / 2;
        int xpxl2 = xend;  // This will be used in the main loop
        int ypxl2 = (int) yend;
        if (steep) {
            result.Add((ypxl2, xpxl2));
            result.Add((ypxl2 + 1, xpxl2));
        }
        else {
            result.Add((xpxl2, ypxl2));
            result.Add((xpxl2, ypxl2 + 1));
        }

        // main loop
        if (steep) {
            for (int x = xpxl1 + 1; x < xpxl2; x++) {
                int y = (int) intery;

                result.Add((y, x));
                result.Add((y + 1, x));
                intery = intery + gradient;
            }
        }
        else {
            for (int x = xpxl1 + 1; x < xpxl2; x++) {
                int y = (int) intery;

                result.Add((x, y));
                result.Add((x, y + 1));
                intery = intery + gradient;
            }
        }

        return result;
    }

    private void RegisterLine(DynamicLine line, int cellX, int cellY) {
        if (this[cellX, cellY] == null)
            this[cellX, cellY] = new List<DynamicLine> { line };
        
        else
            this[cellX, cellY].Add(line);
    }

    public void Clear() {
        for (int i = 0; i < cells.Length; i++) 
            cells[i] = null;
    }

    private List<DynamicLine> this[int x, int y] {
        get => cells[width * y + x];
        set => cells[width * y + x] = value;
    }

    private bool BelongsToGrid(float x, float y) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    // Draw the grid using Unity Gizmos (for debugging):
    public void DrawGizmos(float z) {
        Gizmos.color = Color.black;

        float startX = cornerX, endX = cornerX + width * cellSize;
        float startY = cornerY, endY = cornerY - height * cellSize;

        for (int i = 0; i <= width; i++) {
            float x = startX + i * cellSize;
            Gizmos.DrawLine(Utils.To3D(x, startY, z),
                Utils.To3D(x, endY, z));
        }

        for (int i = 0; i <= height; i++) {
            float y = startY - i * cellSize;
            Gizmos.DrawLine(Utils.To3D(startX, y, z),
                Utils.To3D(endX, y, z));
        }

        // Draw in black each occupied cell:
        for(int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                if (this[x, y] != null) {
                    float xPos = cornerX + cellSize * (x + 0.5f);
                    float yPos = cornerY - cellSize * (y + 0.5f);
                    Gizmos.DrawCube(Utils.To3D(xPos, yPos, z), 
                        Utils.To3D(cellSize, cellSize, 0));
                }
            }
        }
    }

    // Debug function to draw how a grid would look like even if the grid is not instantiated:
    public static void DrawGizmos(float z, int width, int height, float centerX, float centerY, 
        float cellSize) {

        Gizmos.color = Color.black;

        float cornerX = centerX - width * cellSize / 2;
        float cornerY = centerY + height * cellSize / 2;

        float startX = cornerX, endX = cornerX + width * cellSize;
        float startY = cornerY, endY = cornerY - height * cellSize;

        for (int i = 0; i <= width; i++) {
            float x = startX + i * cellSize;
            Gizmos.DrawLine(Utils.To3D(x, startY, z),
                Utils.To3D(x, endY, z));
        }

        for (int i = 0; i <= height; i++) {
            float y = startY - i * cellSize;
            Gizmos.DrawLine(Utils.To3D(startX, y, z),
                Utils.To3D(endX, y, z));
        }
    }
}