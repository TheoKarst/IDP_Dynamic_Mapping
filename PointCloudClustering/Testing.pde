int[] myCreateClusters(PVector[] points, float dth) {
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
