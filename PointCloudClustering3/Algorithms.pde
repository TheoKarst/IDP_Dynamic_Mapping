final float DOUGLAS_PEUCKER_EPSILON = 1;

ArrayList<PVector> DouglasPeucker(ArrayList<PVector> points) {
  return DouglasPeucker(points, 0, points.size()-1, DOUGLAS_PEUCKER_EPSILON);
}

// Run douglas peucker algorithm on the given list of points, between start and end (inclusives):
ArrayList<PVector> DouglasPeucker(ArrayList<PVector> points, int start, int end, float epsilon) {
  PVector u = PVector.sub(points.get(start), points.get(end), new PVector());
  u.normalize();
  
  // Find the point with maximum orthogonal distance:
  int index = -1;
  float maxDistance = 0;
  
  // Tempoary variables to compute the orthogonal distance:
  PVector v = new PVector();
  PVector v_along_u = new PVector();
  
  PVector startPoint = points.get(start);
  for(int i = start+1; i < end; i++) {
    PVector.sub(points.get(i), startPoint, v);
    PVector.mult(u, v.dot(u), v_along_u);
    float distance = PVector.sub(v, v_along_u).mag();
    
    if(distance >= maxDistance) {
      maxDistance = distance;
      index = i;
    }
  }    
  
  if(maxDistance > epsilon) {
    ArrayList<PVector> list1 = DouglasPeucker(points, start, index, epsilon);
    ArrayList<PVector> list2 = DouglasPeucker(points, index, end, epsilon);
    
    list1.remove(list1.size()-1);
    for(PVector point : list2) list1.add(point);
    
    return list1;
  }
  
  else {
    ArrayList<PVector> result = new ArrayList<PVector>();
    result.add(startPoint);
    result.add(points.get(end));
    return result;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ArrayList<PVector> giftWrapping(ArrayList<PVector> points) {
  ArrayList<PVector> hull = new ArrayList<PVector>();
  
  // Find the leftmost point:
  PVector leftmost = points.get(0);
  for(int i = 1; i < points.size(); i++) {
    PVector current = points.get(i);
    if(current.x < leftmost.x)
      leftmost = current;
  }
  
  int bugCounter = 0;
  PVector tmp = new PVector();
  PVector pointOnHull = leftmost;
  do {
    hull.add(pointOnHull);
    PVector endpoint = points.get(0);
    PVector n = new PVector(pointOnHull.y - endpoint.y, endpoint.x - pointOnHull.x);
    
    for(PVector point : points) {
      if(endpoint == pointOnHull || PVector.sub(point, pointOnHull, tmp).dot(n) < 0) {
        endpoint = point;
        n.set(pointOnHull.y - endpoint.y, endpoint.x - pointOnHull.x);
      }
    }
    pointOnHull = endpoint;
    
    bugCounter++;
    if(bugCounter > 100)
      println(leftmost + "; " + pointOnHull);
  }
  while(pointOnHull != leftmost);
  
  hull.add(pointOnHull);
  
  return hull;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ArrayList<PVector> curveRegularization(ArrayList<PVector> curve, float step) {
  ArrayList<PVector> result = new ArrayList<PVector>();
  
  if(curve.size() == 0)
    return result;
    
  PVector pivot = curve.get(0);
  result.add(new PVector().set(pivot));
  step *= step;      // During the algorithm, we are just using step²
  
  int index = 1;
  PVector current, next;
  PVector OB = new PVector();
  float OB_magSq = -1;
  
  while(index < curve.size()) {
    current = next = pivot;
    
    while(index < curve.size()) {
      next = curve.get(index);
      OB_magSq = PVector.sub(next, pivot, OB).magSq();
      
      if(OB_magSq >= step)
        break;
      
      current = next;
      index++;
    }
    
    if(index < curve.size()) {
      PVector BA = PVector.sub(current, next, new PVector());
      float a = BA.magSq();
      float b = OB.dot(BA);
      float c = OB_magSq - step;
      
      float t = (-b - sqrt(b*b - a*c)) / a;
      pivot = BA.mult(t).add(next);
      result.add(pivot);
    }
    else
      result.add(new PVector().set(next));
  }  
  
  return result;
}
