int[] myCreateLabels(PVector[] points, float dth) {
  int labels[] = new int[points.length];
  
  int cluster = 1;
  PVector tmp = new PVector();
  ArrayList<Integer> neighbours = new ArrayList();
  for(int i = 0; i < points.length; i++) {
    // If the point has no label, create a new cluster for this point:
    if(labels[i] == 0) {
      neighbours.add(i);
      labels[i] = cluster;
      
      while(!neighbours.isEmpty()) {
        Integer current = neighbours.remove(neighbours.size()-1);
        for(int j = 0; j < points.length; j++) {
          if(labels[j] == 0 && PVector.sub(points[current], points[j], tmp).mag() <= dth) {
            labels[j] = cluster;
            neighbours.add(j);
          }
        }
      }
      
      cluster++;
    }
  }
  
  return labels;
}


ArrayList<ArrayList<PVector>> myCreateClusters(PVector[] points, float dth) {
  // Create a list of clusters (list of group of points):
  ArrayList<ArrayList<PVector>> clusters = new ArrayList();
  
  // For each point, if it was associated to a cluster or not:
  boolean clustered[] = new boolean[points.length];
  
  PVector tmp = new PVector();
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
          if(!clustered[j] && PVector.sub(points[current], points[j], tmp).mag() <= dth) {
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

void benchmarks() {
  PVector[] points = generateRandomPoints(80000);
  
  // Create benchmarks for fast euclidian clustering:
  int t;
  t = millis(); createClusters(points, 0.1f); t = millis() - t;
  println("Fast Euclidian clustering: " + t / 1000f + "s");
  
  t = millis(); createClusters(points, 5); t = millis() - t;
  println("Fast Euclidian clustering: " + t / 1000f + "s");
  
  t = millis(); createClusters(points, 20); t = millis() - t;
  println("Fast Euclidian clustering: " + t / 1000f + "s");
  
  
  // Create benchmarks for my implementation:
  t = millis(); myCreateClusters(points, 0.1f); t = millis() - t;
  println("Personal clustering: " + t / 1000f + "s");
  
  t = millis(); myCreateClusters(points, 5); t = millis() - t;
  println("Personal clustering: " + t / 1000f + "s");
  
  t = millis(); myCreateClusters(points, 20); t = millis() - t;
  println("Personal clustering: " + t / 1000f + "s");
}

PVector[] generateRandomPoints(int count) {
  PVector[] points = new PVector[count];
  
  for(int i = 0; i < count; i++) {
    points[i] = new PVector(random(100), random(100));
  }
  
  return points;
}
