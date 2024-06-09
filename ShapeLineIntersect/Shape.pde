public class Shape {
  private PVector center;
  private ArrayList<PVector> points = new ArrayList<PVector>();
  private ArrayList<Float> angles = new ArrayList<Float>();
  
  public Shape(PVector center) {
    this.center = center;
  }
  
  public void Draw() {
    noStroke();
    fill(0);
    ellipse(center.x, -center.y, 10, 10);
    
    if(points.size() == 0)
      return;
      
    stroke(0, 0, 255);
    strokeWeight(2);
    PVector prev = points.get(points.size()-1);
    for(PVector point : points) {
      line(prev.x, -prev.y, point.x, -point.y);
      prev = point;
    }
    
    stroke(0);
    strokeWeight(1);
    for(PVector point : points) {
      line(center.x, -center.y, point.x, -point.y);
    }
  }
  
  public void AddPoint(PVector point) {
    points.add(point);
    
    float angle = atan2(point.y - center.y, point.x - center.x);
    if(angle < 0) angle += TWO_PI;
    
    angles.add(angle);
    println("Angle: " + degrees(angle) + "°");
  }
  
  public void Reset() {
    points.clear();
    angles.clear();
  }
  
  public void ComputeIntersections(Line line) {
    if(points.size() < 3)
      return;
      
    println("\nCompute Line intersection:");
    
    // All the indices are modulo n:
    final int n = points.size();
    
    int startPoint, endPoint, direction, count;
    
    // Compute if we have to check the intersections between the edges of the shape and the line in clockwise or anticlockwise order:
    if(line.IsLeftOfLine(center)) {
      direction = 1;
      startPoint = FindSection(line.start) - 1;
      endPoint = FindSection(line.end);
      count = endPoint - startPoint;
      if(startPoint < 0) startPoint += n;
    }
    else {
      direction = -1;
      startPoint = FindSection(line.start);
      endPoint = FindSection(line.end) - 1;
      count = startPoint - endPoint;
      if(endPoint < 0) endPoint += n;
    }
    
    // We have startPoint in [0; n[ and endPoint in [0; n[. Thus count is in ]-n; n[. We just have to keep count in [0; n[:
    if(count < 0) count += n;
    
    int index = startPoint;
    PVector current = points.get(index);
    boolean isLeft = line.IsLeftOfLine(current);
    
    // If the start point of the line is outside the shape:
    boolean startPointOutside = true;
    
    ArrayList<Float> intersections = new ArrayList<Float>();
    for(int i = 0; i < count; i++) {
      index += direction;
      if(index < 0) index += n;
      else if(index >= n) index -= n;
      PVector next = points.get(index);
      boolean nextIsLeft = line.IsLeftOfLine(next);
      
      // Compute startPointOutside:
      if(i == 0) {
        // Rotate (next - current) to point outside the shape:
        float nX, nY;
        if(direction == 1) { nX = next.y - current.y; nY = current.x - next.x; }
        else {               nX = current.y - next.y; nY = next.x - current.x; }   
        
        // Compute the dot product between n and (line.start - current). If the dot product is greater than 0, the point is outside the shape:
        startPointOutside = nX * (line.start.x - current.x) + nY * (line.start.y - current.y) >= 0;
      }
      
      // If both points are on the same side of the line, there is no intersection:
      if(nextIsLeft == isLeft) {
        println("Line: " + index + " => No intersection !");
      }
      else {
        float distance = line.IntersectDistance(current, next);
        if(distance >= 0 && distance <= 1)
          intersections.add(distance);
          
        println("Line: " + index + " => Intersection: " + distance);
      }
      
      current = next;
      isLeft = nextIsLeft;
    }
    
    line.UpdateState(intersections, startPointOutside);
  }
  
  // Return the index of a point with an angle strictly greater than the given point:
  private int FindSection(PVector point) {
    float pointAngle = atan2(point.y - center.y, point.x - center.x);
    if(pointAngle < 0) pointAngle += TWO_PI;
    
    int section = 0;
    while(section < angles.size() && angles.get(section) < pointAngle)
      section++;
    
    return section == angles.size() ? 0 : section;
  }
}
