class Curve {
  private ArrayList<PVector> points;
  private final float stroke;
  private final float regularizationStep;
  
  private boolean isLoop = false;             // If true, the first point of the curve is the same as the last one
  
  public Curve(ArrayList<PVector> initialPoints, float stroke, float regularizationStep) {
    this.points = curveRegularization(initialPoints, regularizationStep);
    this.stroke = stroke;
    this.regularizationStep = regularizationStep;
  }
  
  public void Draw() {
    if(points.size() < 2)
      return;
      
    strokeWeight(5);
    stroke(isLoop ? color(0, 255, 0) : color(255, 0, 0));
    PVector previous = points.get(0);
    for(int i = 1; i < points.size(); i++) {
      PVector current = points.get(i);
      line(previous.x, previous.y, current.x, current.y);
      previous = current;
    }
    strokeWeight(2);
  }
  
  public void DrawContour() {
    if(points.size() < 2)
      return;
      
    PVector[] units = computeUnits();
    
    noFill();
    stroke(0);
    strokeWeight(2);
    PVector prev = null;
    for(int i = 0; i < units.length; i++) {
      PVector curr = points.get(i);
      
      if(prev != null) {
        PVector n = PVector.sub(curr, prev, new PVector()).normalize().rotate(HALF_PI).mult(stroke);
        line(prev.x + n.x, prev.y + n.y, curr.x + n.x, curr.y + n.y);
        line(prev.x - n.x, prev.y - n.y, curr.x - n.x, curr.y - n.y);
      }
      
      ellipse(curr.x, curr.y, 2*stroke, 2*stroke);
      line(curr.x - stroke * units[i].y, curr.y + stroke * units[i].x, 
           curr.x + stroke * units[i].y, curr.y - stroke * units[i].x);
           
      prev = curr;
    }
  }
  
  private PVector[] computeUnits() {
    final int n = points.size();      // assert n >= 2
    PVector units[] = new PVector[n];
    
    // Compute the first and last unit vector along the curve:
    units[0] = PVector.sub(points.get(1), points.get(0), new PVector()).normalize();
    units[n-1] = PVector.sub(points.get(n-1), points.get(n-2), new PVector()).normalize();
    
    PVector prevUnit = units[0];
    for(int i = 1; i < n-1; i++) {
      PVector nextUnit = PVector.sub(points.get(i+1), points.get(i), new PVector()).normalize(); 
      
      units[i] = new PVector((prevUnit.x + nextUnit.x)/2, (prevUnit.y + nextUnit.y)/2).normalize();
      prevUnit = nextUnit;
    }
    
    return units;
  }
  
  public boolean[] MatchCurve(ArrayList<PVector> otherCurve) {
    
    // For each point of the other curve, return if the point can be matched by this curve:
    boolean[] match = new boolean[otherCurve.size()];
    
    if(points.size() < 2)
      return match;
      
    PVector[] units = computeUnits();
    
    int thisFirstMatchIndex = -1;
    int otherFirstMatchIndex = -1;
    int lastMatchLastSegment = -1;
    int otherEndLoopIndex = -1;
    
    for(int i = 0; i < otherCurve.size(); i++) {
      int matchIndex = belongIndex(otherCurve.get(i), units);
      
      // If there is a match between the other curve and this one:
      if(matchIndex != -1) {
        match[i] = true;
        
        if(thisFirstMatchIndex == -1) {
          thisFirstMatchIndex = matchIndex;
          otherFirstMatchIndex = i;
        }
        
        // If we matched the last segment of this curve:
        if(matchIndex == points.size() - 2) {
          lastMatchLastSegment = i;
        }
        
        // Else, if we matched the first segment of this curve and already passed through the last segment, this curve is a loop:
        else if(!isLoop && matchIndex == 0 && lastMatchLastSegment != -1) {
          isLoop = true;
          otherEndLoopIndex = i;
        }
      }
      else
        match[i] = false;
    }
    
    if(isLoop) {
      for(int i = lastMatchLastSegment + 1; i < otherEndLoopIndex; i++) {
        points.add(otherCurve.get(i));
        match[i] = true;
      }
      
      // Duplicate first point to close the loop:
      points.add(points.get(0));
    }
    else {
      ArrayList<PVector> newCurve = new ArrayList<PVector>();
      
      if(thisFirstMatchIndex == 0) {
        for(int i = 0; i < otherFirstMatchIndex; i++) {
          newCurve.add(otherCurve.get(i));
          match[i] = true;
        }
      }
      
      for(PVector point : points)
        newCurve.add(point);
        
      if(lastMatchLastSegment != -1) {
        for(int i = lastMatchLastSegment + 1; i < otherCurve.size(); i++) {
          newCurve.add(otherCurve.get(i));
          match[i] = true;
        }
      }
      
      points = newCurve;
    }
    
    // Regularize the curve:
    points = curveRegularization(points, regularizationStep);
    
    return match;
  }
  
  private int belongIndex(PVector point, PVector[] units) {    
    for(int i = 0; i < points.size()-1; i++) {
      if(isInside(point, points.get(i), points.get(i+1), units[i], units[i+1]))
        return i;
    }
    
    return -1;
  }
  
  // Return if the point is inside the segment [A, B]:
  private boolean isInside(PVector point, PVector A, PVector B, PVector unitA, PVector unitB) {
    float strokeSq = stroke * stroke;
    
    PVector AP = PVector.sub(point, A, new PVector());
    PVector BP = PVector.sub(point, B, new PVector());
    
    if(AP.dot(unitA) < 0 || BP.dot(unitB) > 0)
      return false;
      
    if(AP.magSq() <= strokeSq || BP.magSq() <= strokeSq)
      return true;
    
    // Dot product with the vector AB:
    PVector tmp = PVector.sub(B, A, new PVector());
    float dotU = AP.dot(tmp);
    
    if(dotU >= 0 && dotU <= tmp.magSq()) {
      tmp.normalize();
      tmp.set(-tmp.y, tmp.x);
      
      // Dot product with the unit vector orthogonal to AB:
      if(abs(AP.dot(tmp)) <= stroke)
        return true;
    }
    return false;
  }
}
