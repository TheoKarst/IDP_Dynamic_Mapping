class ClusterManager {
  // Parameters for the cluster algorithm:
  private final int pointClusterLifetime;
  private final float rectClusterMargin;
  
  private StaticCluster[] staticClusters;
  private ArrayList<DynamicCluster> dynamicClusters;
  
  public ClusterManager(int pointClusterLifetime, float rectClusterMargin) {
    this.pointClusterLifetime = pointClusterLifetime;
    this.rectClusterMargin = rectClusterMargin;
    
    this.dynamicClusters = new ArrayList<DynamicCluster>();
  }
  
  public void UpdateClusters(PVector[] points) {
    staticClusters = buildStaticClusters(points);
    
    // Match each dynamic cluster with a static cluster. As static clusters represent current object estimates, dynamic clusters
    // are used to track clusters between frames. We have to match dynamic clusters to static clusters to see if previous objects are
    // still there on the current frame (dynamic clusters are unable to manage cluster creation or destruction, while static clusters
    // are able to do so):
    ArrayList<DynamicCluster> notMatchedClusters = new ArrayList<DynamicCluster>();
    for(DynamicCluster dynamicCluster : dynamicClusters) {
      StaticCluster match = null;
      
      for(StaticCluster staticCluster : staticClusters) {
        if(staticCluster.matches(dynamicCluster)) {
          match = staticCluster;
          break;
        }
      }
      
      dynamicCluster.matchCluster(match);
      if(match == null) 
        notMatchedClusters.add(dynamicCluster);
      else
        match.addDynamicClusterMatch(dynamicCluster);
    }
    
    // Update all the dynamic clusters:
    for(DynamicCluster cluster : dynamicClusters)
      cluster.Update();
    
    // Now, each static cluster that doesn't contains any dynamic cluster means a new object appeared in the scene (that wasn't here on the previous frame).
    // In this situation, a new dynamic cluster has to be created to follow this new object.
    ArrayList<DynamicCluster> newDynamicClusters = new ArrayList<DynamicCluster>();
    for(StaticCluster cluster : staticClusters) {
      cluster.Update(newDynamicClusters);
      // if(!cluster.hasMatch()) {
      //   newDynamicClusters.add(cluster.createAndMatchDynamicCluster(clusterColors[dynamicClusters.size()]));
      // }
    }
    
    // Remove the dynamic clusters that weren't matched with a static cluster for long enough:
    for(DynamicCluster cluster : notMatchedClusters)
      if(cluster.getFrameCountNoMatch() <= pointClusterLifetime)
        newDynamicClusters.add(cluster);
        
    // Update the set of dynamic clusters:
    dynamicClusters = newDynamicClusters;
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
      
      if(PVector.dist(previous, current) > MAX_DISTANCE) {
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
    if(PVector.dist(previous, points[0]) > MAX_DISTANCE) {
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
