PVector[] DouglasPeucker(PVector[] points, float epsilon) {
  return DouglasPeucker(points, 0, points.length-1, epsilon);
}

// Run douglas peucker algorithm on the given list of points, between start and end (inclusives):
PVector[] DouglasPeucker(PVector[] points, int start, int end, float epsilon) {
  PVector u = PVector.sub(points[start], points[end], new PVector());
  u.normalize();
  
  // Find the point with maximum orthogonal distance:
  int index = -1;
  float maxDistance = 0;
  
  // Tempoary variables to compute the orthogonal distance:
  PVector v = new PVector();
  PVector v_along_u = new PVector();
  
  for(int i = start+1; i < end; i++) {
    PVector.sub(points[i], points[start], v);
    PVector.mult(u, v.dot(u), v_along_u);
    float distance = PVector.sub(v, v_along_u).mag();
    
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
