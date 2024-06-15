ArrayList<PVector> curveRegularization2(ArrayList<PVector> curve, float stepRadius, int minLookAhead) {
  ArrayList<PVector> result = new ArrayList<PVector>();
  
  if(curve.size() == 0)
    return result;
    
  PVector pivot = curve.get(0);
  result.add(pivot);
  
  float stepRadiusSq = stepRadius * stepRadius;      // During the algorithm, we are just using stepRadius²
  
  int index = 1;
  PVector OB = new PVector();
  float OB_magSq = -1;
  
  while(index < curve.size()) {
    int lookahead = 1;
    int lastExitRadius = -1;
    boolean previousInsideRadius = true;
    
    while(index < curve.size() && (previousInsideRadius || lookahead <= minLookAhead)) {
      PVector next = curve.get(index);
      OB_magSq = PVector.sub(next, pivot, OB).magSq();
      
      if(OB_magSq >= stepRadiusSq) {
        if(previousInsideRadius) {
          lastExitRadius = index;
          
          if(lookahead >= minLookAhead) break;
        }
        
        previousInsideRadius = false;
      }
      else
        previousInsideRadius = true;
      
      lookahead++;
      index++;
    }
    
    if(lastExitRadius != -1) {
      index = lastExitRadius;
      
      PVector next = curve.get(lastExitRadius);
      OB_magSq = PVector.sub(next, pivot, OB).magSq();
      
      PVector BA = PVector.sub(curve.get(lastExitRadius-1), next, new PVector());
      float a = BA.magSq();
      float b = OB.dot(BA);
      float c = OB_magSq - stepRadiusSq;
      
      float t = (-b - sqrt(b*b - a*c)) / a;
      pivot = BA.mult(t).add(next);
      result.add(pivot);
    }
    else
      result.add(new PVector().set(curve.get(curve.size()-1)));
  }  
  
  return result;
}
