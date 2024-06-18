ArrayList<PVector> curveRegularization4(ArrayList<PVector> curve, float stepRadius, int minLookAhead, float epsilon) {
  ArrayList<PVector> result = new ArrayList<PVector>();
  
  if(curve.size() == 0)
    return result;
    
  int lastPivotIndex = 0;
  PVector pivot = curve.get(0);
  //println("Pivot: 0");
  
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
        
      //println("Lookahead: " + lookahead + ". Point " + index + " is " + (previousInsideRadius ? "inside" : "outside") + " radius.");
      
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
      
      PVector nextPivot = BA.mult(t).add(next);
      // print("Building pivot before " + lastExitRadius + " ?");
      
      // Before adding this new point in the result, we check if we are not missing an important point:
      boolean abort = false;
      
      if(lastPivotIndex + 1 < index) {
        PVector u = new PVector(pivot.y - nextPivot.y, nextPivot.x - pivot.x).normalize();
        
        // Find the point with maximum orthogonal distance:
        int maxIndex = -1;
        float maxDistance = 0;
        
        PVector tmp = new PVector();
        for(int i = lastPivotIndex + 1; i < index; i++) {
          float distance = PVector.sub(curve.get(i), pivot, tmp).dot(u);
          
          if(distance > maxDistance) {
            maxIndex = i;
            maxDistance = distance;
          }
        }
        
        if(maxDistance > epsilon) {
          lastPivotIndex = maxIndex;
          pivot = curve.get(maxIndex);
          result.add(pivot);
          index = maxIndex + 1;
          abort = true;
        }
      }
      
      if(!abort) {
        lastPivotIndex = lastExitRadius-1;
        pivot = nextPivot;
        result.add(pivot);
      }
    }
    else
      result.add(new PVector().set(curve.get(curve.size()-1)));
  }  
  
  return result;
}
