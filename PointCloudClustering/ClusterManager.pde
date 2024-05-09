class ClusterManager {
  private final color clusterColors[];                     // List of colors that are used to draw the clusters
  
  // Parameters for the cluster algorithm:
  private final int minPointsPerRectCluster;
  private final float pointClustersVisibility;
  private final int pointClusterLifetime;
  private final float rectClusterMargin;
  
  private ArrayList<ArrayList<PVector>> clustersPoints;    // For each frame, group points by distance
  private ArrayList<RectCluster> rectClusters;             // Use rectangle clusters to represent the previous groups of points
  private ArrayList<PointCluster> pointClusters;           // Use point clusters to track clusters between frames
  
  public ClusterManager(color[] clusterColors, int minPointsPerRectCluster, float pointClustersVisibility, int pointClusterLifetime, float rectClusterMargin) {
    this.clusterColors = clusterColors;
    
    this.minPointsPerRectCluster = minPointsPerRectCluster;
    this.pointClustersVisibility = pointClustersVisibility;
    this.pointClusterLifetime = pointClusterLifetime;
    this.rectClusterMargin = rectClusterMargin;
    
    this.pointClusters = new ArrayList<PointCluster>();
  }
  
  public void UpdateClusters(PVector[] points) {
    // Cluster points by distance:
    clustersPoints = clusterPointsByDistance(points, 20);
    
    // Create rectangle clusters to represent to sets of points:
    rectClusters = new ArrayList<RectCluster>();
    for(int i = 0; i < clustersPoints.size(); i++) {
      ArrayList<PVector> mPoints = clustersPoints.get(i);
      
      if(mPoints.size() >= minPointsPerRectCluster) {
        RectCluster cluster = new RectCluster(mPoints, rectClusterMargin);
        rectClusters.add(cluster);
      }
    }
    
    // Match each point cluster with a rectangle cluster. As rectangle clusters represent current object estimates, point clusters
    // are used to track clusters between frames. We have to match point clusters to rect clusters to see if previous objects are
    // still there on the current frame (point clusters are unable to manage cluster creation or destruction, while rectangle clusters
    // are able to do so):
    for(PointCluster pointCluster : pointClusters) {
      RectCluster match = null;
      
      for(RectCluster rectCluster : rectClusters) {
        if(rectCluster.contains(pointCluster.position())) {
          match = rectCluster;
          break;
        }
      }
      
      pointCluster.matchCluster(match);
      if(match != null) match.addPointClusterMatch(pointCluster);
    }
    
    // Update all the point clusters:
    for(PointCluster cluster : pointClusters)
      cluster.Update();
    
    // Now, each rectangle cluster that doesn't contains any point cluster means a new object appeared in the scene (that wasn't here on the previous frame).
    // In this situation, a new point cluster has to be created to follow this new object.
    ArrayList<PointCluster> newPointClusters = new ArrayList<PointCluster>();
    for(RectCluster cluster : rectClusters) {
      if(!cluster.hasMatch()) {
        newPointClusters.add(cluster.createAndMatchPointCluster(clusterColors[pointClusters.size()], pointClustersVisibility));
      }
    }
    
    // Remove the point clusters that weren't matched with a rect cluster for long enough:
    for(PointCluster cluster : pointClusters)
      if(cluster.getFrameCountNoMatch() <= pointClusterLifetime)
        newPointClusters.add(cluster);
        
    // Update the set of point clusters:
    pointClusters = newPointClusters;
  }
  
  public void Draw(boolean showPoints, boolean showRectClusters, boolean showPointClusters) {
    // Draw all the points of the current frame:
    if(showPoints) {
      stroke(0); fill(0);
      for(int i = 0; i < clustersPoints.size(); i++) {
        for(PVector point : clustersPoints.get(i))
          ellipse(point.x, point.y, 5, 5);
      }
    }
    
    // Draw rectangle clusters, matching the current frame:
    if(showRectClusters)
      for(RectCluster cluster : rectClusters)
        cluster.Draw();
        
    // Draw point clusters, representing the objects across different frames:
    if(showPointClusters)
      for(PointCluster cluster : pointClusters)
        cluster.Draw();
  }
  
  // Create recursively groups of points, merging points by distance:
  private ArrayList<ArrayList<PVector>> clusterPointsByDistance(PVector[] points, float distance) {
    // Create a list of clusters (list of group of points):
    ArrayList<ArrayList<PVector>> clusters = new ArrayList();
    
    // For each point, if it was associated to a cluster or not:
    boolean clustered[] = new boolean[points.length];
    
    ArrayList<Integer> neighbours = new ArrayList();
    for(int i = 0; i < points.length; i++) {
      // If the point was not associated to a cluster, create a new cluster for this point:
      if(!clustered[i]) {
        ArrayList<PVector> currentCluster = new ArrayList();
        
        neighbours.add(i);
        currentCluster.add(points[i]);
        clustered[i] = true;
        
        while(!neighbours.isEmpty()) {
          Integer current = neighbours.remove(neighbours.size()-1);
          for(int j = 0; j < points.length; j++) {
            if(!clustered[j] && PVector.dist(points[current], points[j]) <= distance) {
              currentCluster.add(points[j]);
              clustered[j] = true;
              neighbours.add(j);
            }
          }
        }
        
        clusters.add(currentCluster);
      }
    }
    
    return clusters;
  }
}
