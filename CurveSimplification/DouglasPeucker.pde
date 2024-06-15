PVector[] DouglasPeucker(PVector[] points, float epsilon) {
  return DouglasPeucker(points, 0, points.length-1, epsilon);
}

ArrayList<PVector> DouglasPeucker(ArrayList<PVector> points, float epsilon) {
  return DouglasPeucker(points, 0, points.size()-1, epsilon);
}


// Run douglas peucker algorithm on the given list of points, between start and end (inclusives):
PVector[] DouglasPeucker(PVector[] points, int start, int end, float epsilon) {
  PVector u = new PVector(points[start].y - points[end].y, points[end].x - points[start].x).normalize();
  
  // Find the point with maximum orthogonal distance:
  int index = -1;
  float maxDistance = 0;
  
  PVector tmp = new PVector();
  for(int i = start+1; i < end; i++) {
    float distance = abs(PVector.sub(points[i], points[start], tmp).dot(u));
    
    if(distance >= maxDistance) {
      maxDistance = distance;
      index = i;
    }
  }
  
  if(maxDistance > epsilon) {
    PVector list1[] = DouglasPeucker(points, start, index, epsilon);
    PVector list2[] = DouglasPeucker(points, index, end, epsilon);
    
    PVector result[] = new PVector[list1.length + list2.length - 1];
    for(int i = 0; i < list1.length-1; i++) result[i] = list1[i];
    for(int i = 0; i < list2.length; i++) result[list1.length+i-1] = list2[i];
    
    return result;
  }
  else
    return new PVector[]{points[start], points[end]};
}

// Run douglas peucker algorithm on the given list of points, between start and end (inclusives):
ArrayList<PVector> DouglasPeucker(ArrayList<PVector> points, int start, int end, float epsilon) {
  PVector startPoint = points.get(start), endPoint = points.get(end);
  PVector u = new PVector(startPoint.y - endPoint.y, endPoint.x - startPoint.x).normalize();
  
  // Find the point with maximum orthogonal distance:
  int index = -1;
  float maxDistance = 0;
  
  PVector tmp = new PVector();
  for(int i = start+1; i < end; i++) {
    float distance = abs(PVector.sub(points.get(i), startPoint, tmp).dot(u));
    
    if(distance >= maxDistance) {
      maxDistance = distance;
      index = i;
    }
  }
  
  if(maxDistance > epsilon) {
    ArrayList<PVector> list1 = DouglasPeucker(points, start, index, epsilon);
    ArrayList<PVector> list2 = DouglasPeucker(points, index, end, epsilon);
    
    ArrayList<PVector> result = new ArrayList<PVector>(list1.size() + list2.size() - 1);
    for(int i = 0; i < list1.size()-1; i++) result.add(list1.get(i));
    for(int i = 0; i < list2.size(); i++) result.add(list2.get(i));
    
    return result;
  }
  else {
    ArrayList<PVector> result = new ArrayList<PVector>(2);
    result.add(startPoint);
    result.add(endPoint);
    return result;
  }
}
