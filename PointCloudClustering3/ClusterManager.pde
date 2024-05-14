final float MAX_STATIC_CLUSTER_DISTANCE = 100;

class ClusterManager {
  private StaticCluster[] staticClusters;
  private ArrayList<DynamicCluster> dynamicClusters;
  
  public ClusterManager() {
    this.dynamicClusters = new ArrayList<DynamicCluster>();
  }
  
  public void UpdateClusters(PVector[] points) {
    staticClusters = buildStaticClusters(points);
    
    // Match the dynamic clusters from the previous frame, with the points of the current frame:
    for(DynamicCluster dynamicCluster : dynamicClusters)
      dynamicCluster.MatchCluster(staticClusters);
  }
  
  public void Draw(boolean showStaticClusters, boolean showDynamicClusters) {
    // Draw static clusters, matching the current frame:
    if(showStaticClusters)
      for(StaticCluster cluster : staticClusters)
        cluster.Draw();
        
    // Draw dynamic clusters, representing the objects across different frames:
    if(showDynamicClusters)
      for(DynamicCluster cluster : dynamicClusters)
        cluster.Draw();
  }
  
  StaticCluster[] buildStaticClusters(PVector points[]) {
    if(points.length <= 1)
      return new StaticCluster[0];
      
    ArrayList<ArrayList<PVector>> pointsClusters = new ArrayList<ArrayList<PVector>>();
    ArrayList<PVector> currentCluster = new ArrayList<PVector>();
    
    PVector previous = points[0];
    for(int i = 1; i < points.length; i++) {
      PVector current = points[i];
      
      if(PVector.dist(previous, current) > MAX_STATIC_CLUSTER_DISTANCE) {
        if(currentCluster.size() > 0) {
          pointsClusters.add(currentCluster);
          currentCluster = new ArrayList<PVector>();
        }
      }
      else {
        if(currentCluster.size() == 0)
          currentCluster.add(previous);
        
        currentCluster.add(current);
      }
      
      previous = current;
    }
    
    // To complete the shape, we also have to check the distance between the point n and the point 0. If they are connected,
    // they should belong to the same cluster:
    if(PVector.dist(previous, points[0]) > MAX_STATIC_CLUSTER_DISTANCE) {
      if(currentCluster.size() > 0)
        pointsClusters.add(currentCluster);
    }
    else {
      if(currentCluster.size() == 0)
        currentCluster.add(previous);
      
      // If the first point already belongs to a cluster, merge this cluster with the current one:
      if(pointsClusters.size() > 0 && pointsClusters.get(0).get(0) == points[0]) {
        for(PVector point : pointsClusters.get(0))
          currentCluster.add(point);
          
        pointsClusters.set(0, currentCluster);  // Replace the first cluster with the merge we just created
      }
      
      // Else, add the first point into the last cluster, and add the cluster to the list:
      else {
        currentCluster.add(points[0]);
        pointsClusters.add(currentCluster);
      }
    }
    
    // Now we can build static clusters from the point clusters we just created:
    StaticCluster[] staticClusters = new StaticCluster[pointsClusters.size()];
    for(int i = 0; i < staticClusters.length; i++)
      staticClusters[i] = new StaticCluster(pointsClusters.get(i), rectClusterMargin);
      
    return staticClusters;
  }
}
