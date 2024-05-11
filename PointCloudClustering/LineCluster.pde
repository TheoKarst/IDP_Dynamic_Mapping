// Testing: Instead of using the point cloud as a random set of points, we instead use the knowledge that LIDAR measures are sorted,
// and draw a 2D shape around the sensor. This way, to check if two points belongs to the same shape, instead of checking the distance
// between all the points, we just check the distance between a point i and its successor i+1
final float MAX_DISTANCE = 20;

void drawContours(PVector points[], color[] myColors) {
  if(points.length <= 1)
    return;
    
  ArrayList<ArrayList<PVector>> lineClusters = new ArrayList<ArrayList<PVector>>();
  ArrayList<PVector> currentCluster = new ArrayList<PVector>();
  
  PVector previous = points[0];
  for(int i = 1; i < points.length; i++) {
    PVector current = points[i];
    
    if(PVector.dist(previous, current) > MAX_DISTANCE) {
      if(currentCluster.size() > 0) {
        lineClusters.add(currentCluster);
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
      lineClusters.add(currentCluster);
  }
  else {
    if(currentCluster.size() == 0)
      currentCluster.add(previous);
    
    // If the first point already belongs to a cluster, merge this cluster with the current one:
    if(lineClusters.size() > 0 && lineClusters.get(0).get(0) == points[0]) {
      for(PVector point : lineClusters.get(0))
        currentCluster.add(point);
        
      lineClusters.set(0, currentCluster);  // Replace the first cluster with the merge we just created
    }
    
    // Else, add the first point into the last cluster, and add the cluster to the list:
    else {
      currentCluster.add(points[0]);
      lineClusters.add(currentCluster);
    }
  }
  
  
  // Now we can draw the shapes:
  strokeWeight(5);
  for(int i = 0; i < lineClusters.size(); i++) {
    ArrayList<PVector> cluster = lineClusters.get(i);
    previous = cluster.get(0);
    
    stroke(myColors[i]);
    for(int j = 1; j < cluster.size(); j++) {
      PVector current = cluster.get(j);
      line(previous.x, previous.y, current.x, current.y);
      previous = current;
    }
  }
  strokeWeight(1);
}
