ArrayList<PVector> curveRegularization1(ArrayList<PVector> curve, float step) {
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
