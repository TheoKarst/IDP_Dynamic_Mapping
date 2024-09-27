using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

/// <summary>
/// Class to build a dynamic map of an environment using lines and circles
/// </summary>
public class GeometryMapping {
    
    private GeometryMapParams parameters;

    // Match to each observation (rho, theta) from the LIDAR a point in world space coordinate:
    private List<Point> currentPoints;

    // MatchGrid used for a more efficient matching between lines:
    private MatchGrid matchGrid;

    // Current lines and circles used to represent the environment:
    private List<DynamicLine> modelLines = new List<DynamicLine>();
    private List<Circle> modelCircles = new List<Circle>();

    // Wipe shape used during the current frame, to remove inconsistent primitives:
    private WipeShape currentWipeShape;

    // For debugging: List of lines that were built during this frame:
    private List<DynamicLine> debugCurrentLines = new List<DynamicLine>();

    private float lastTimeUpdate = -1;

    // Used for logging data:
    private const int MAX_LOG_LINES = 10;
    private StreamWriter lineLogFile;
    private Dictionary<int, int> mapModelIdToColumns;
    private int logLine = 0;

    /// <summary>
    /// Instantiates the mapping algorithm using geometric primitives
    /// </summary>
    /// <param name="parameters">All the parameters used by the algorithm</param>
    public GeometryMapping(GeometryMapParams parameters) {
        this.parameters = parameters;

        this.matchGrid = new MatchGrid(
            parameters.width, parameters.height, 
            parameters.centerX, parameters.centerY, 
            parameters.cellSize);

        if(parameters.LogLinesData) {
            mapModelIdToColumns = new Dictionary<int, int>();

            lineLogFile = File.CreateText("./Assets/Data/logs/lines_data.csv");
            lineLogFile.Write("Time (s);Iteration;");
            for (int i = 0; i < MAX_LOG_LINES; i++)
                lineLogFile.Write("matchCount;isStatic;"
                    + "rho (m);theta (deg);dRho (m/s);dTheta(deg/s);"
                    + "eRho (m);eTheta (deg);eDRho (m/s);eDTheta (deg/s);");
            lineLogFile.WriteLine();
        }
    }

    /// <summary>
    /// Updates the lines and circles in the model from the observations of a sensor
    /// </summary>
    /// <param name="worldSensorPose">World pose of the sensor which made the observations</param>
    /// <param name="model">Model of the vehicle</param>
    /// <param name="vehicleState">State estimate of the vehicle</param>
    /// <param name="stateCovariance">State covariance of the vehicle</param>
    /// <param name="observations">Observations of the sensor</param>
    /// <param name="currentTime">Current time in seconds</param>
    public void UpdateModel(Pose2D worldSensorPose, VehicleModel model, VehicleState vehicleState, Matrix<double> stateCovariance, Observation[] observations, float currentTime) {
        float deltaTime = lastTimeUpdate == -1 ? 0 : currentTime - lastTimeUpdate;
        lastTimeUpdate = currentTime;

        // Clear the previous grid map, and add in it the current model lines:
        matchGrid.Clear();
        foreach(DynamicLine line in modelLines)
            matchGrid.RegisterLine(line);
        
        // Get points from the LIDAR:
        currentPoints = ComputePoints(model, vehicleState, stateCovariance, observations);

        // Build the Wipe Shape:
        currentWipeShape = BuildWipeShape(worldSensorPose, model, vehicleState, observations);

        // Then use the points to perform lines and circles extraction:
        List<DynamicLine> lines; List<Circle> circles;
        (lines, circles) = ExtractPrimitives(currentPoints);

        // For debugging:
        if(parameters.drawCurrentLines) {
            debugCurrentLines.Clear();
            foreach(DynamicLine line in lines) {
                DynamicLine copy = new DynamicLine(line, line.beginPoint, line.endPoint);
                copy.lineColor = Color.yellow;
                debugCurrentLines.Add(copy);
            }
        }

        // List<WipeTriangle> triangles = UpdateModelLines(sensorPosition, lines);
        UpdateModelLines(lines, currentWipeShape, deltaTime);

        // Use the circles from the current frame to update the model circles:
        // UpdateModelCircles(circles, triangles);
        UpdateModelCircles(circles, currentWipeShape, deltaTime);

        // Debug.Log("Points: " + currentPoints.Count + "; Lines: " + modelLines.Count 
        //    + "; Circles: " + modelCircles.Count);
    }

    /// <summary>
    /// Draws the components that have been created for the mapping: lines, circles, wipe shape and match grid
    /// </summary>
    public void DrawGizmos() {
        const float height = 0.2f;

        if (parameters.drawCurrentLines && debugCurrentLines != null) {
            foreach(DynamicLine line in debugCurrentLines)
                line.DrawGizmos(height, false);
        }

        if (parameters.drawPoints && currentPoints != null 
            && currentPoints.Count > 0 && currentPoints[0] != null) {

            foreach (Point point in currentPoints)
                point.DrawGizmos(height, parameters.drawPointsError);
        }

        if (parameters.drawLines && modelLines != null) {
            foreach (DynamicLine line in modelLines)
                line.DrawGizmos(height, parameters.drawSpeedEstimates);
        }

        if(parameters.drawCircles && modelCircles != null) {
            foreach (Circle circle in modelCircles)
                circle.DrawGizmos(height, parameters.drawSpeedEstimates);
        }

        if(parameters.drawWipeShape && currentWipeShape != null) {
            currentWipeShape.DrawGizmos(height);
        }

        if(parameters.drawMatchGrid && matchGrid != null) {
            matchGrid.DrawGizmos(height);
        }
    }

    /// <summary>
    /// Use the LIDAR observations and the vehicle state estimate from the Kalman Filter
    /// to compute the estimated position and covariance matrix of all the observations 
    /// of the LIDAR in world space
    /// </summary>
    private static List<Point> ComputePoints(VehicleModel model, VehicleState vehicleState, Matrix<double> stateCovariance, Observation[] observations) {
        List<Point> points = new List<Point>();

        // All the observations are supposed to come from the same LIDAR:
        int lidarIndex = observations[0].lidarIndex;

        // Compute the position of all the observations that are not out of range:
        (Vector<double>[] Xps, Matrix<double>[] Cps) 
            = model.ComputeObservationsPositionsEstimates(vehicleState, stateCovariance, observations, lidarIndex);

        for(int i = 0; i < observations.Length; i++) {
            if (!observations[i].outOfRange) {
                float x = (float)Xps[i][0];
                float y = (float)Xps[i][1];
                float theta = observations[i].theta;

                points.Add(new Point(x, y, theta, Cps[i]));
            }
        }

        return points;
    }

    /// <summary>
    /// Given the observations of a LIDAR, use these observations to build a wipe-shape
    /// representing the free-space of the sensor
    /// </summary>
    private WipeShape BuildWipeShape(Pose2D worldSensorPose, VehicleModel model, VehicleState vehicleState, Observation[] observations) {
        // 1. Compute the local position of the observations after clamping:
        Vector2[] positions = new Vector2[observations.Length];
        float[] angles = new float[observations.Length];
        for (int i = 0; i < observations.Length; i++) {
            float r = observations[i].r, theta = observations[i].theta;

            if (r > parameters.clampDistance)
                r = parameters.clampDistance;

            positions[i] = new Vector2(r * Mathf.Cos(theta), r * Mathf.Sin(theta));
            angles[i] = theta;
        }

        // 2. Apply alpha filter to the shape:
        bool[] remove = WipeShapeUtils.AlphaFilter(positions, angles, parameters.alpha * Mathf.Deg2Rad);

        // 3. Apply Douglas Peucker algorithm to the remaining points:
        remove = WipeShapeUtils.DouglasPeucker(positions, parameters.epsilon, remove);

        // Count the points we need to keep:
        int count = 0;
        for (int i = 0; i < remove.Length; i++)
            if (!remove[i])
                count++;

        // 4. Finally, compute the world space position of the points subset:
        Vector2[] positionsSubset = new Vector2[count];
        float[] anglesSubset = new float[count];

        for(int i = remove.Length-1; i >= 0; i--) {
            if (remove[i])
                continue;

            Observation observation = observations[i];
            if(observation.r > parameters.clampDistance)
                observation.r = parameters.clampDistance;

            count--;
            positionsSubset[count] = model.ComputeObservationPositionEstimate(vehicleState, observation);
            anglesSubset[count] = worldSensorPose.angle + observation.theta;
        }

        return new WipeShape(new Vector2(worldSensorPose.x, worldSensorPose.y), positionsSubset, anglesSubset);
    }

    /// <summary>
    /// Builds lines and circles to represent observed points
    /// </summary>
    private (List<DynamicLine>, List<Circle>) ExtractPrimitives(List<Point> points) {
        List<DynamicLine> extractedLines = new List<DynamicLine>();
        List<Circle> extractedCircles = new List<Circle>();

        // We first try to match the first point with a line, then with a circle.
        // If we are building a line, we should have lineBuilder == null.
        // If we are building a circle, we should have circleBuilder == null.
        LineBuilder lineBuilder = null;
        CircleBuilder circleBuilder = null;

        foreach (Point currentPoint in points) {

            // Initialisation: this should be executed just for the first non null point:
            if(lineBuilder == null && circleBuilder == null) {
                lineBuilder = new LineBuilder(currentPoint);
                continue;
            }

            // 1. If we are currently building a line:
            if(lineBuilder != null) {
                Point previousPoint = lineBuilder.GetLastPoint();

                // Try to match the current point with the current line:
                bool condition1 = Point.Dist(previousPoint, currentPoint) <= parameters.PointCriticalDistance;
                bool condition2 = lineBuilder.PointsCount() < 3 || lineBuilder.DistanceFrom(currentPoint) <= parameters.LineCriticalDistance;
                bool condition3 = Point.AngularDifference(previousPoint, currentPoint) <= parameters.CriticalAlphaRadians;
                
                // If the three conditions are met, we can add the point to the line:
                if (condition1 && condition2 && condition3) {
                    lineBuilder.AddPoint(currentPoint);
                    continue;
                }

                // Else, if the current line is long enough to be extracted, then extract it, and add the current
                // point in a new line:
                else if (lineBuilder.PointsCount() >= parameters.LineMinPoints 
                    && lineBuilder.Length() >= parameters.LineMinLength) {

                    extractedLines.Add(lineBuilder.Build());
                    lineBuilder = new LineBuilder(currentPoint);
                    continue;
                }

                // Otherwise, convert the current line into a circle cluster, and continue to process this circle:
                else {
                    circleBuilder = lineBuilder.ToCircle();
                    lineBuilder = null;
                }
            }

            // 2. If we are currently building a circle:

            // If the current point can be added to the current circle, add it:
            if (circleBuilder.DistanceFrom(currentPoint) <= parameters.CircleCriticalDistance) {
                circleBuilder.AddPoint(currentPoint);
            }

            // Otherwise, extract the current circle and add the current point in a new line:
            else {
                if (circleBuilder.PointsCount() >= parameters.CircleMinPoints) {
                    extractedCircles.Add(circleBuilder.Build());
                }

                circleBuilder = null;
                lineBuilder = new LineBuilder(currentPoint);
            }
        }

        // Finally, extract the current line or current circle if necessary:
        if (lineBuilder != null 
            && lineBuilder.PointsCount() >= parameters.LineMinPoints 
            && lineBuilder.Length() >= parameters.LineMinLength) {

            extractedLines.Add(lineBuilder.Build());
        }
        else if(circleBuilder != null 
            && circleBuilder.PointsCount() >= parameters.CircleMinPoints) {

            extractedCircles.Add(circleBuilder.Build());
        }

        return (extractedLines, extractedCircles);
    }

    /// <summary>
    /// Uses the observed lines to update the lines in the model
    /// </summary>
    /// <param name="observedLines">Lines extracted from the current observations</param>
    /// <param name="wipeShape">Wipe-shape representing the free area of the sensor</param>
    /// <param name="deltaTime">Elapsed time in seconds since the last update</param>
    private void UpdateModelLines(List<DynamicLine> observedLines, WipeShape wipeShape, float deltaTime) {
        List<DynamicLine> newLines = new List<DynamicLine>();

        foreach (DynamicLine line in modelLines) {
            // Reset the color of the lines from the model:
            line.lineColor = Color.red;

            // Predict the new state of the lines, knowing the elapsed time since the last update:
            if(!parameters.StaticLines)
                line.PredictState(deltaTime, parameters.LineProcessNoiseError, parameters.LinesFriction);

            // Finally, find which sections of the line are valid or not, using the wipe shape:
            wipeShape.UpdateLineValidity(line);
        }

        // Try to match the current lines with the lines in the model:
        for (int i = 0; i < observedLines.Count; i++) {
            DynamicLine currentLine = observedLines[i];

            // The best match we found in the model, as well as the
            // norm distance between this line and the one in the model:
            DynamicLine bestMatch = null;
            float minNormDistance = -1;

            foreach (DynamicLine modelLine in matchGrid.FindNeighbors(currentLine)) {

                // First test: check if the line from the model is a good match candidate:
                if (currentLine.IsMatchCandidate(modelLine,
                    parameters.LineMaxMatchAngleRadians,
                    parameters.LineMaxMatchOrthogonalDistance,
                    parameters.LineMaxMatchParallelDistance)) {

                    // Second test: Compute the Mahalanobis distance between the two lines:
                    float normDistance = currentLine.NormDistanceFromModel(modelLine);

                    if(bestMatch == null || normDistance < minNormDistance) {
                        bestMatch = modelLine;
                        minNormDistance = normDistance;
                    }
                }
            }

            // If a match was found and is near enough, use the current line to
            // update the matched line in the model:
            if(bestMatch != null && minNormDistance < 5) {
                bestMatch.lineColor = Color.blue;       // Blue color for matched lines
                bestMatch.UpdateLineUsingMatch(currentLine, 
                    parameters.LineValidityExtent,
                    parameters.StaticMaxRhoDerivative,
                    parameters.StaticMaxThetaDerivativeRadians,
                    parameters.MinMatchesToConsiderStatic);
            }

            // Otherwise, we just add this new line to the model:
            else {
                currentLine.lineColor = Color.green;    // Green color for new lines
                newLines.Add(currentLine);
            }
        }

        // Keep only the valid parts of the lines from the model:
        foreach (DynamicLine line in modelLines) {
            line.AddValidParts(newLines,
                parameters.LineMinLength,           // Minimum length of the lines we are allowed to keep
                parameters.MaxLineErrorRhoSq,       // Maximum value for the variance of rho
                parameters.MaxLineErrorThetaSq,     // Maximum value for the variance of theta
                parameters.InitialisationSteps);    // Number of matches before checking the two above parameters
        }

        // Replace the previous lines with the updated ones:
        modelLines = newLines;

        // Debug:
        if(parameters.LogLinesData && lineLogFile != null)
            LogModelLines();
    }

    /// <summary>
    /// Writes data of the model lines in a csv file. This can be useful to draw curves in Excel, showing
    /// how well lines are tracked
    /// </summary>
    public void LogModelLines() {
        const int DATA_COUNT = 10;

        if(modelLines.Count == 0)
            return;

        float[] lineData = new float[DATA_COUNT * MAX_LOG_LINES];

        foreach (DynamicLine line in modelLines) {
            int index;
            if (mapModelIdToColumns.ContainsKey(line.id)) {
                index = mapModelIdToColumns[line.id];
            }
            else if (mapModelIdToColumns.Count < MAX_LOG_LINES) {
                index = DATA_COUNT * mapModelIdToColumns.Count;
                mapModelIdToColumns.Add(line.id, index);
            }
            else
                continue;

            DynamicLine.LineState state = line.GetState();
            Matrix<double> covariance = line.GetCovariance();
            float eRho    = Mathf.Sqrt((float)covariance[0, 0]);
            float eTheta  = Mathf.Sqrt((float)covariance[1, 1]);
            float eDRho   = Mathf.Sqrt((float)covariance[2, 2]);
            float eDTheta = Mathf.Sqrt((float)covariance[3, 3]);

            lineData[index + 0] = line.matchCount;
            lineData[index + 1] = line.isStatic ? 1 : 0;
            lineData[index + 2] = state.rho;
            lineData[index + 3] = state.theta * Mathf.Rad2Deg;
            lineData[index + 4] = state.dRho;
            lineData[index + 5] = state.dTheta * Mathf.Rad2Deg;
            lineData[index + 6] = eRho;
            lineData[index + 7] = eTheta * Mathf.Rad2Deg;
            lineData[index + 8] = eDRho;
            lineData[index + 9] = eDTheta * Mathf.Rad2Deg;
        }

        lineLogFile.Write(Time.realtimeSinceStartup + ";" + logLine + ";");
        for (int i = 0; i < lineData.Length; i++)
            lineLogFile.Write(lineData[i] + ";");

        lineLogFile.WriteLine();
        lineLogFile.Flush();

        logLine++;
    }

    /// <summary>
    /// Uses the observed circles to update the circles in the model
    /// </summary>
    /// <param name="observedCircles">Circles extracted from the current observations</param>
    /// <param name="wipeShape">Wipe-shape representing the free area of the sensor</param>
    /// <param name="deltaTime">Elapsed time in seconds since the last update</param>
    public void UpdateModelCircles(List<Circle> observedCircles, WipeShape wipeShape, float deltaTime) {
        List<Circle> newCircles = new List<Circle>();

        foreach (Circle circle in modelCircles) {
            // Drawing: Reset the color of the model circles to red:
            circle.circleColor = Color.red;

            // Predict the current state of the circle:
            circle.PredictState(deltaTime, parameters.CirclesFriction);

            // Compute if the circle is consistent with the free space of the wipe-shape:
            wipeShape.UpdateCircleValidity(circle);
        }

        // Try to match the current circles with the circles in the model:
        foreach (Circle circle in observedCircles) {

            // Check if the observed circle is far enough from the lines already in the model:
            List<DynamicLine> neighboringLines = matchGrid.FindNeighbors(circle);

            if (!circle.IsFarFromLines(neighboringLines, parameters.MinOrthogonalDistanceToLines))
                continue;

            // If this is the case, we try to match this circle with a circle in the model:
            Circle bestMatch = null;
            float minDistance = -1;

            foreach (Circle matchCandidate in modelCircles) {
                float distance = circle.DistanceFrom(matchCandidate);

                if (bestMatch == null || distance < minDistance) {
                    bestMatch = matchCandidate;
                    minDistance = distance;
                }
            }

            // If a match is found, use this circle to update the match state estimate:
            if (bestMatch != null && minDistance <= parameters.CircleMaxMatchDistance) {
                bestMatch.UpdateCircleUsingMatch(circle);

                // If a circle is matched with a current circle, then it's valid:
                bestMatch.isValid = true;
                bestMatch.circleColor = Color.blue;
            }

            // Else, just add the circle to the model:
            else {
                newCircles.Add(circle);
                circle.circleColor = Color.green;
            }
        }

        // Keep only the valid circles in the model:
        foreach (Circle circle in modelCircles)
            if (circle.isValid)
                newCircles.Add(circle);

        modelCircles = newCircles;
    }
}
