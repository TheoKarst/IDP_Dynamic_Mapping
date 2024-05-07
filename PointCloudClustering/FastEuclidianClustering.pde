public int[] createClusters(PVector[] points, float dth) {
  int[] labels = new int[points.length];
  
  int segLab = 1;
  for(int i = 0; i < labels.length; i++) {
    if(labels[i] == 0) {
      ArrayList<Integer> PVV = findNeighbours(points[i], points, dth);
      int minNonZeroLabel = findMinNonZeroLabel(PVV, labels);
      
      int minSegLab = minNonZeroLabel != -1 ? min(minNonZeroLabel, segLab) : segLab;
      // print("MinNonZero: " + minNonZeroLabel + "; SegLab: " + segLab + "; MinSegLab: " + minSegLab + "; Neighbours of: " + i + ": "); printList(PVV);
      
      // Segment merge loop:
      for(Integer j : PVV) {
        int labelJ = labels[j];
        
        if(labelJ > minSegLab) {
          for(int k = 0; k < points.length; k++) {
            if(labels[k] == labelJ)
              labels[k] = minSegLab;
          }
        }
        
        labels[j] = minSegLab;
      }
      
      segLab++;
    }
    
    // printList(labels);
  }
  
  return labels;
}

public ArrayList<Integer> findNeighbours(PVector point, PVector[] points, float dth) {
  ArrayList<Integer> neighbours = new ArrayList<Integer>();
  
  PVector tmp = new PVector();
  for(int i = 0; i < points.length; i++) {
    if(PVector.sub(points[i], point, tmp).mag() <= dth)
      neighbours.add(i);
  }
  
  return neighbours;
}

// Return the points with the minimum label that is not zero. If all the points have a label of 0, return -1:
public int findMinNonZeroLabel(ArrayList<Integer> points, int[] labels) {
  int minNonZeroLabel = -1;
  
  for(Integer point : points) {
    int label = labels[point];
    
    if(label != 0 && (minNonZeroLabel == -1 || label < minNonZeroLabel))
      minNonZeroLabel = label;
  }
  
  return minNonZeroLabel;
}

void printList(int[] list) {
  print("[");
  for(int i = 0; i < list.length; i++) {
    print(list[i]);
    if(i+1 < list.length) print(", ");
  }
  println("]");
}

void printList(ArrayList<Integer> list) {
  print("[");
  for(int i = 0; i < list.size(); i++) {
    print(list.get(i));
    if(i+1 < list.size()) print(", ");
  }
  println("]");
}
