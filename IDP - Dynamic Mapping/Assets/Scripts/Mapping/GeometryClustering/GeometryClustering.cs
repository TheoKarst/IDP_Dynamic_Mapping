using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

public class GeometryClustering {
    private GeometryClusterParams parameters;

    // Match to each observation (rho, theta) from the LIDAR a point in world space coordinate:
    private List<Point> currentPoints;

    // Current lines and circles used to represent the environment:
    private List<DynamicLine> modelLines = new List<DynamicLine>();
    private List<Circle> modelCircles = new List<Circle>();

    // For debugging: List of lines that were built during this frame:
    private List<DynamicLine> debugCurrentLines = new List<DynamicLine>();

    private GridMap gridMap;
    private WipeShape currentWipeShape;

    private float lastTimeUpdate = -1;

    // TEST:
    private int newLineCount = 0;

    public GeometryClustering(GeometryClusterParams parameters, GridMap gridMap) {
        this.parameters = parameters;
        this.gridMap = gridMap;
    }

    public void UpdateModel(Pose2D sensorPose, VehicleModel model, VehicleState vehicleState, Matrix<double> stateCovariance, AugmentedObservation[] observations, float currentTime) {
        float deltaTime = lastTimeUpdate == -1 ? 0 : currentTime - lastTimeUpdate;
        lastTimeUpdate = currentTime;

        Vector2[] positions;

        // Clear the previous grid map, and add in it the current model lines:
        Profiler.BeginSample("Grid Map Update");
        gridMap.Clear();
        foreach(DynamicLine line in modelLines)
            gridMap.RegisterLine(line);
        Profiler.EndSample();

        // Get points from the LIDAR:
        Profiler.BeginSample("Compute points");
        (positions, currentPoints) = ComputePoints(model, vehicleState, stateCovariance, observations);
        Profiler.EndSample();

        // Build the Wipe Shape:
        Profiler.BeginSample("Build Wipe Shape");
        // currentWipeShape = BuildWipeShape(sensorPose, positions);
        currentWipeShape = BuildWipeShape3(sensorPose, model, vehicleState, observations);
        Profiler.EndSample();

        // Then use the points to perform lines and circles extraction:
        List<DynamicLine> lines; List<Circle> circles;

        Profiler.BeginSample("Cluster Extraction");
        (lines, circles) = ClusterExtraction(currentPoints);
        Profiler.EndSample();

        // For debugging:
        if(parameters.drawCurrentLines) {
            debugCurrentLines.Clear();
            foreach(DynamicLine line in lines) {
                DynamicLine copy = new DynamicLine(line, line.beginPoint, line.endPoint);
                copy.lineColor = Color.yellow;
                debugCurrentLines.Add(copy);
            }
        }

        Profiler.BeginSample("Update Model Lines");
        // List<WipeTriangle> triangles = UpdateModelLines(sensorPosition, lines);
        UpdateModelLines(lines, currentWipeShape, deltaTime);
        Profiler.EndSample();

        // Use the circles from the current frame to update the model circles:
        Profiler.BeginSample("Update Model Circles");
        // UpdateModelCircles(circles, triangles);
        UpdateModelCircles(circles, currentWipeShape);
        Profiler.EndSample();

        // Debug.Log("Points: " + currentPoints.Count + "; Lines: " + modelLines.Count 
        //    + "; Circles: " + modelCircles.Count);
    }

    public void DrawGizmos(bool drawCurrentLines, bool drawPoints, 
        bool drawLines, bool drawCircles, bool drawWipeShape) {

        const float height = 0.2f;

        if (drawCurrentLines && debugCurrentLines != null) {
            foreach(DynamicLine line in debugCurrentLines)
                line.DrawGizmos(height);
        }

        if (drawPoints && currentPoints != null && currentPoints[0] != null) {
            foreach (Point point in currentPoints)
                point.DrawGizmos(height);
        }

        if (drawLines && modelLines != null) {
            foreach (DynamicLine line in modelLines)
                line.DrawGizmos(height);
        }

        if(drawCircles && modelCircles != null) {
            foreach (Circle circle in modelCircles)
                circle.DrawGizmos(height);
        }

        if(drawWipeShape && currentWipeShape != null) {
            currentWipeShape.DrawGizmos(height);
        }
    }

    /*
    // Use the LIDAR observations and the vehicle state estimate from the Kalman Filter
    // to compute the estimated position of all the observations of the LIDAR in world space:
    private static (Vector2[] positions, List<Point> points) ComputePoints(VehicleModel model, VehicleState vehicleState, Matrix<double> stateCovariance, AugmentedObservation[] observations) {
        Vector2[] positions = new Vector2[observations.Length];
        List<Point> points = new List<Point>();
        
        for(int i = 0; i < observations.Length; i++) {
            Observation observation = new Observation(observations[i].r, observations[i].theta);

            // If the observation was out of the range of the LIDAR, still compute
            // the estimated position of the clamped observation, that may be useful
            // to build the WipeShape (we don't need the corresponding
            // covariance estimate):
            if (observations[i].outOfRange) {
                positions[i] = model.ComputeObservationPositionEstimate(vehicleState, observation);
            }

            // Else, compute the observation position and corresponding covariance
            // matrix to update the model lines and circles:
            else {
                Vector<double> position; Matrix<double> covariance;
                (position, covariance) =
                    model.ComputeObservationPositionEstimate(vehicleState, stateCovariance, observation);

                float x = (float)position[0], y = (float)position[1], theta = observation.theta;

                positions[i] = new Vector2(x, y);
                points.Add(new Point(x, y, theta, covariance));
            }                
        }

        return (positions, points);
    }*/

    // Use the LIDAR observations and the vehicle state estimate from the Kalman Filter
    // to compute the estimated position of all the observations of the LIDAR in world space:
    private static (Vector2[] positions, List<Point> points) ComputePoints(VehicleModel model, VehicleState vehicleState, Matrix<double> stateCovariance, AugmentedObservation[] observations) {
        Vector2[] positions = new Vector2[observations.Length];
        List<Point> points = new List<Point>();

        (Vector<double>[] Xps, Matrix<double>[] Cps) 
            = model.ComputeObservationsPositionsEstimates(vehicleState, stateCovariance, observations, observations[0].lidarIndex);

        for(int i = 0; i < positions.Length; i++) {
            if (observations[i].outOfRange) {
                positions[i] = new Vector2((float)Xps[i][0], (float)Xps[i][1]);
            }
            else {
                float x = (float)Xps[i][0];
                float y = (float)Xps[i][1];
                float theta = observations[i].theta;

                positions[i] = new Vector2(x, y);
                points.Add(new Point(x, y, theta, Cps[i]));
            }
        }

        return (positions, points);
    }

    /*
    private WipeShape BuildWipeShape(Pose2D sensorPose, Vector2[] currentPoints) {
        List<Vector2> points = LidarUtils.ComputeWipeShapePoints(currentPoints,
            parameters.stepRadius, parameters.minLookAhead, parameters.epsilon);

        // Convert into an array:
        Vector2[] pointsArray = new Vector2[points.Count];
        for (int i = 0; i < points.Count; i++)
            pointsArray[i] = points[i];

        // Run Douglas Peucker to get less points for the shape:
        Vector2[] filtered = LidarUtils.DouglasPeucker(pointsArray, parameters.douglasEpsilon);
        
        // Build the WipeShape:
        return new WipeShape(new Vector2(sensorPose.x, sensorPose.y), filtered);
    }

    private WipeShape BuildWipeShape2(Pose2D sensorPose, VehicleModel model, VehicleState vehicleState, AugmentedObservation[] observations) {
        List<int> indices = WipeShapeUtils.HighPassSubsample(observations, parameters.stepAngle * Mathf.Deg2Rad);

        Vector2[] points = new Vector2[indices.Count];
        for(int i = 0; i < indices.Count; i++) {
            Observation observation = observations[indices[i]].ToObservation();
            points[i] = model.ComputeObservationPositionEstimate(vehicleState, observation);
        }

        // Run Douglas Peucker to get less points for the shape:
        Vector2[] filtered = LidarUtils.DouglasPeucker(points, parameters.douglasEpsilon);

        return new WipeShape(new Vector2(sensorPose.x, sensorPose.y), filtered);
    }*/

    private WipeShape BuildWipeShape3(Pose2D sensorPose, VehicleModel model, VehicleState vehicleState, AugmentedObservation[] observations) {
        
        // Compute the world space position of all the given observations:
        Vector2[] positions = new Vector2[observations.Length];
        float[] angles = new float[observations.Length];
        for (int i = 0; i < positions.Length; i++) {
            Observation observation = observations[i].ToObservation();

            // Clamp the observation:
            if(observation.r > parameters.clampDistance)
                observation.r = parameters.clampDistance;

            positions[i] = model.ComputeObservationPositionEstimate(vehicleState, observation);
            angles[i] = observation.theta;
        }

        // Apply alpha filter to the shape:
        int[] indices = WipeShapeUtils.AlphaFilter(positions, angles, parameters.alpha * Mathf.Deg2Rad);

        // Apply Douglas Peucker algorithm to the remaining points:
        indices = LidarUtils.DouglasPeuckerIndices(positions, indices, 0, indices.Length-1, parameters.epsilon);

        Vector2[] filteredPoints = new Vector2[indices.Length];
        float[] filteredAngles = new float[indices.Length];
        for(int i = 0; i < indices.Length; i++) {
            filteredPoints[i] = positions[indices[i]];
            filteredAngles[i] = angles[indices[i]];
        }

        return new WipeShape(new Vector2(sensorPose.x, sensorPose.y), filteredPoints, filteredAngles);
    }

    // Cluster extraction (lines and circles):
    private (List<DynamicLine>, List<Circle>) ClusterExtraction(List<Point> points) {
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

    /*
    private List<WipeTriangle> UpdateModelLines(Vector2 sensorPosition, List<Line> currentLines) {

        // List of wipe triangles built from the current lines:
        List<WipeTriangle> wipeTriangles = new List<WipeTriangle>();

        // Drawing: Reset the color of the model lines to red:
        foreach (Line line in modelLines) line.lineColor = Color.red;

        // Try to match the current lines with the lines in the model:
        for (int i = 0; i < currentLines.Count; i++) {
            Line line = currentLines[i];

            // Build the wipe triangle for this line:
            WipeTriangle wipeTriangle = line.BuildWipeTriangle(sensorPosition, WipeTriangleExtent, WipeTriangleInsideAngleMarginRadians);
            if(wipeTriangle != null)
                wipeTriangles.Add(wipeTriangle);

            // The best matching line we found in the model, as well as the
            // distance between this line and the one in the model:
            Line bestMatch = null;
            float minDistance = -1;

            Profiler.BeginSample("Lines matching");
            foreach (Line matchCandidate in modelLines) {
                // First test: compare the angle difference and endpoints distance between the two lines:
                if (line.IsMatchCandidate(matchCandidate, LineMaxMatchAngleRadians, LineMaxEndpointMatchDistance)) {

                    // Second test: Compute the Mahalanobis distance between the lines:
                    if (line.ComputeNormDistance(matchCandidate) < 5) {

                        // Last step: Find the nearest line among the remaining candidates:
                        // TODO: Check that this implementation matches the paper description:
                        Profiler.BeginSample("Compute Center Distance");
                        float centerDistance = line.ComputeCenterDistance(matchCandidate);
                        Profiler.EndSample();

                        if (bestMatch == null || centerDistance < minDistance) {
                            bestMatch = matchCandidate;
                            minDistance = centerDistance;
                        }
                    }
                }
            }
            Profiler.EndSample();

            // If a match is found and near enough, use this line to update the match state estimate:
            Profiler.BeginSample("Line Update");
            if (bestMatch != null && minDistance <= LineMaxMatchDistance) {
                bestMatch.lineColor = Color.blue;       // Blue color for matched lines
                bestMatch.UpdateLineUsingMatching(line);

                // Update all the model (except the matching line) using the wipe triangle of this line:
                if (wipeTriangle != null)
                    wipeTriangle.UpdateLines(modelLines, bestMatch, LineMinLength);

                // modelLines.Add(bestMatch);
            }

            // Else, just add this new line to the model:
            else {
                line.lineColor = Color.green;       // Green color for new lines

                if (wipeTriangle != null)
                    wipeTriangle.UpdateLines(modelLines, null, LineMinLength);

                modelLines.AddLast(line);
                //Debug.Log("New line (" + (++newLineCount) + ") !");
            }
            Profiler.EndSample();
        }

        // List of wipe triangles that we built, that will be used to remove inconsistent circles:
        return wipeTriangles;
    }*/

    // Update the lines using a wipe shape instead of wipe triangles:
    /*private void UpdateModelLines(List<Line> currentLines, WipeShape wipeShape) {
        // bool DEBUG_NEW_LINE = false;

        List<Line> newLines = new List<Line>();

        // Drawing: Reset the color of the model lines to red:
        foreach (Line line in modelLines) line.lineColor = Color.red;

        // Update the lines in the model using the wipe shape:
        Profiler.BeginSample("Wipe Shape Line Update");
        wipeShape.UpdateLines(modelLines);
        Profiler.EndSample();

        // Try to match the current lines with the lines in the model:
        for (int i = 0; i < currentLines.Count; i++) {
            Line line = currentLines[i];

            // The best matching line we found in the model, as well as the
            // distance between this line and the one in the model:
            Line bestMatch = null;
            float minDistance = -1;

            Profiler.BeginSample("Lines matching");
            string logMsg = "Line " + i + ": [" + line.LogParams() + "]\n";
            // foreach (Line matchCandidate in modelLines) {
            foreach(Line matchCandidate in gridMap.FindNeighbors(line)) {
                logMsg += "[model: " + matchCandidate.LogParams() + "]\t";
                logMsg += line.LogMatchCandidate(matchCandidate, LineMaxMatchAngleRadians, parameters.LineMaxEndpointMatchDistance) + "\t";
                logMsg += line.LogNormDistance(matchCandidate) + "\t";
                logMsg += "[" + Utils.ScientificNotation(line.ComputeCenterDistance(matchCandidate)) + "]";

                // First test: compare the angle difference and endpoints distance between the two lines:
                if (line.IsMatchCandidate(matchCandidate, LineMaxMatchAngleRadians, parameters.LineMaxEndpointMatchDistance)) {

                    // Second test: Compute the Mahalanobis distance between the lines:
                    if (line.ComputeNormDistance(matchCandidate) < 5) {

                        // Last step: Find the nearest line among the remaining candidates:
                        // TODO: Check that this implementation matches the paper description:
                        Profiler.BeginSample("Compute Center Distance");
                        float centerDistance = line.ComputeCenterDistance(matchCandidate);
                        Profiler.EndSample();

                        if (bestMatch == null || centerDistance < minDistance) {
                            bestMatch = matchCandidate;
                            minDistance = centerDistance;
                            logMsg += ": Possible match !";
                        }
                    }
                }
                logMsg += "\n";
            }
            Profiler.EndSample();

            // If a match is found and near enough, use this line to update the match state estimate:
            Profiler.BeginSample("Line Update");
            if (bestMatch != null && minDistance <= parameters.LineMaxMatchDistance) {
                bestMatch.lineColor = Color.blue;       // Blue color for matched lines
                bestMatch.UpdateLineUsingMatching(line);
                // logMsg += "=> Match found !";
            }

            // Else, just add this new line to the model:
            else {
                line.lineColor = Color.green;       // Green color for new lines
                newLines.Add(line);
                Debug.Log("New line (" + (++newLineCount) + "):\n" + logMsg);
                // DEBUG_NEW_LINE = true;
            }

            Profiler.EndSample();
        }

        // Keep only the valid parts of the lines from the model:
        foreach (Line line in modelLines)
            line.AddValidParts(newLines, parameters.LineMinLength);

        modelLines = newLines;

        // if(DEBUG_NEW_LINE)
        //     Debug.Break();
    }*/

    private void UpdateModelLines(List<DynamicLine> currentLines, WipeShape wipeShape, float deltaTime) {
        List<DynamicLine> newLines = new List<DynamicLine>();

        foreach (DynamicLine line in modelLines) {
            // Reset the color of the lines from the model:
            line.lineColor = Color.red;

            // Predict the new state of the lines, knowing the elapsed time since the last update:
            line.PredictState(deltaTime, parameters.LineProcessNoiseError);

            // Finally, find which sections of the line are valid or not, using the wipe shape:
            wipeShape.UpdateLineValidity(line);
        }

        // Try to match the current lines with the lines in the model:
        for (int i = 0; i < currentLines.Count; i++) {
            DynamicLine currentLine = currentLines[i];

            bool matchFound = false;
            foreach (DynamicLine modelLine in gridMap.FindNeighbors(currentLine)) {

                // First test: check if the line from the model is a good match candidate:
                if (currentLine.IsMatchCandidate(modelLine,
                    parameters.LineMaxMatchAngleRadians,
                    parameters.LineMaxMatchOrthogonalDistance,
                    parameters.LineMaxMatchParallelDistance)) {

                    // Second test: Check the Mahalanobis distance between the two lines:
                    if (currentLine.NormDistanceFromModel(modelLine) < 5) {

                        // As soon as there is a match, use this line to update the model line:
                        modelLine.lineColor = Color.blue;   // Blue color for matched lines
                        modelLine.UpdateLineUsingMatch(currentLine, 
                            parameters.LineObservationError, 
                            parameters.LineValidityExtent);

                        matchFound = true;
                        break;
                    }
                }
            }

            // If no match was found, we just add this line to the model:
            if(!matchFound) {
                currentLine.lineColor = Color.green;    // Green color for new lines
                newLines.Add(currentLine);
                Debug.Log("New line ! Count: " + (++newLineCount));
            }
        }

        // Keep only the valid parts of the lines from the model:
        foreach (DynamicLine line in modelLines)
            line.AddValidParts(newLines, parameters.LineMinLength);

        // Replace the previous lines with the updated ones:
        modelLines = newLines;
    }

    /*
    public void UpdateModelCircles(List<Circle> currentCircles, List<WipeTriangle> wipeTriangles) {
        List<Circle> newCircles = new List<Circle>();

        // Drawing: Reset the color of the model circles to red:
        foreach (Circle circle in modelCircles) circle.circleColor = Color.red;

        // Try to match the current circles with the circles in the model:
        foreach (Circle circle in currentCircles) {
            Circle bestMatch = null;
            float minDistance = -1;

            foreach(Circle matchCandidate in modelCircles) {
                float distance = circle.DistanceFrom(matchCandidate);

                if(bestMatch == null || distance < minDistance) {
                    bestMatch = matchCandidate;
                    minDistance = distance;
                }
            }

            // If a match is found, use this circle to update the match state estimate:
            if (bestMatch != null && minDistance <= CircleMaxMatchDistance) {
                bestMatch.UpdateCircleUsingMatching(circle);
                bestMatch.circleColor = Color.blue;
            }

            // Else, just add the circle to the model:
            else {
                newCircles.Add(circle);
                circle.circleColor = Color.green;
            }
        }

        // Add the new lines in the model:
        foreach (Circle circle in newCircles)
            modelCircles.Add(circle);

        // Use the wipe triangles of the new lines to delete inconsistent circles:
        foreach(WipeTriangle triangle in wipeTriangles) 
            modelCircles = triangle.UpdateCircles(modelCircles);
    }
    */

    public void UpdateModelCircles(List<Circle> currentCircles, WipeShape wipeShape) {
        List<Circle> newCircles = new List<Circle>();

        // Drawing: Reset the color of the model circles to red:
        foreach (Circle circle in modelCircles) circle.circleColor = Color.red;

        // Update the circles in the model using the wipe shape:
        Profiler.BeginSample("Wipe Shape Circle Update");
        wipeShape.UpdateCircles(modelCircles);
        Profiler.EndSample();

        // Try to match the current circles with the circles in the model:
        foreach (Circle circle in currentCircles) {
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
                bestMatch.UpdateCircleUsingMatching(circle);

                // If a circle is matched with a current circle, then it's valid:
                bestMatch.UpdateValidity(true);
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
