ArrayList<PVector> giftWrapping(ArrayList<PVector> points) {
  ArrayList<PVector> hull = new ArrayList<PVector>();
  
  // Find the leftmost point:
  PVector leftmost = points.get(0);
  for(int i = 1; i < points.size(); i++) {
    PVector current = points.get(i);
    if(current.x < leftmost.x)
      leftmost = current;
  }
  
  PVector tmp = new PVector();
  PVector pointOnHull = leftmost;
  do {
    hull.add(pointOnHull);
    println(pointOnHull);
    
    PVector endpoint = points.get(0);
    PVector n = new PVector(pointOnHull.y - endpoint.y, endpoint.x - pointOnHull.x);
    
    for(PVector point : points) {
      if(endpoint == pointOnHull || PVector.sub(point, pointOnHull, tmp).dot(n) > 0) {
        endpoint = point;
        n.set(pointOnHull.y - endpoint.y, endpoint.x - pointOnHull.x);
      }
    }
    pointOnHull = endpoint;
  }
  while(pointOnHull != leftmost);
  
  return hull;
}

ArrayList<PVector> curveRegularization(ArrayList<PVector> curve, float step) {
  ArrayList<PVector> result = new ArrayList<PVector>();
  
  if(curve.size() == 0)
    return result;
    
  PVector pivot = curve.get(0);
  result.add(pivot);
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
