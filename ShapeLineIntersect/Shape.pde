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
      ellipse(center.x, -center.y, 5, 5);
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
  
  //public void ComputeIntersectionsWrong(Line line) {
  //  if(points.size() < 3)
  //    return;
      
  //  println("\nCompute Line intersection:");
    
  //  // All the indices are modulo n:
  //  final int n = points.size();
    
  //  int startPoint, endPoint, direction, count;
    
  //  // Compute if we have to check the intersections between the edges of the shape and the line in clockwise or anticlockwise order:
  //  if(line.DistanceOf(center) >= 0) {
  //    direction = 1;
  //    startPoint = FindSection(line.start) - 1;
  //    endPoint = FindSection(line.end);
  //    count = endPoint - startPoint;
  //    if(startPoint < 0) startPoint += n;
  //  }
  //  else {
  //    direction = -1;
  //    startPoint = FindSection(line.start);
  //    endPoint = FindSection(line.end) - 1;
  //    count = startPoint - endPoint;
  //    if(endPoint < 0) endPoint += n;
  //  }
    
  //  // We have startPoint in [0; n[ and endPoint in [0; n[. Thus count is in ]-n; n[. We just have to keep count in [0; n[:
  //  if(count < 0) count += n;
    
  //  int index = startPoint;
  //  PVector current = points.get(index);
  //  boolean isLeft = line.DistanceOf(current) >= 0;
    
  //  // If the start point of the line is outside the shape:
  //  boolean startPointOutside = true;
    
  //  ArrayList<Float> intersections = new ArrayList<Float>();
  //  for(int i = 0; i < count; i++) {
  //    index += direction;
  //    if(index < 0) index += n;
  //    else if(index >= n) index -= n;
  //    PVector next = points.get(index);
  //    boolean nextIsLeft = line.DistanceOf(next) >= 0;
      
  //    // Compute startPointOutside:
  //    if(i == 0) {
  //      // Rotate (next - current) to point outside the shape:
  //      float nX, nY;
  //      if(direction == 1) { nX = next.y - current.y; nY = current.x - next.x; }
  //      else {               nX = current.y - next.y; nY = next.x - current.x; }   
        
  //      // Compute the dot product between n and (line.start - current). If the dot product is greater than 0, the point is outside the shape:
  //      startPointOutside = nX * (line.start.x - current.x) + nY * (line.start.y - current.y) >= 0;
  //    }
      
  //    // If both points are on the same side of the line, there is no intersection:
  //    if(nextIsLeft == isLeft) {
  //      println("Line: " + index + " => No intersection !");
  //    }
  //    else {
  //      float distance = line.IntersectDistance(current, next);
  //      if(distance >= 0 && distance <= 1)
  //        intersections.add(distance);
          
  //      println("Line: " + index + " => Intersection: " + distance);
  //    }
      
  //    current = next;
  //    isLeft = nextIsLeft;
  //  }
    
  //  line.UpdateState(intersections, startPointOutside);
  //}
  
  // Corrected version of the line/shape intersection algorithm: now it should also work for line points belonging to the shape contour:
  public void ComputeIntersectionsCorrected(Line line) {
    if(points.size() < 3)
      return;
      
    println("\nCompute Line intersection:");
    
    // All the indices are modulo n:
    final int n = points.size();
    
    int startSection, endSection, direction, count;
    
    // Compute if we have to check the intersections between the edges of the shape and the line in clockwise or counter-clockwise order:
    if(line.DistanceOf(center) >= 0) {
      // Turn counter-clockwise:
      direction = 1;
      
      startSection = FindSectionLower(line.start);
      endSection = FindSectionUpper(line.end);
      count = endSection - startSection;
      if(startSection < 0) startSection += n;
    }
    else {
      // Turn clockwise:
      direction = -1;
      
      startSection = FindSectionUpper(line.start);
      endSection = FindSectionLower(line.end);
      count = startSection - endSection;
      if(endSection < 0) endSection += n;
    }
    
    // We have startPoint in [0; n[ and endPoint in [0; n[. Thus count is in ]-n; n[. We just have to keep count in ]0; n]:
    if(count <= 0) count += n;
    
    println("Start section: " + startSection + ", endSection: " + endSection + ", direction: " + direction + ", count: " + count);
    
    int index = startSection;
    PVector current = points.get(index);
    boolean isLeft = line.DistanceOf(current) >= 0;    
    
    // If the start point of the line is outside the shape (the values doesn't matter since they will be replaced during the first loop):
    boolean startPointOutside = true;
    boolean currentSideUnknown = true;
    
    ArrayList<Float> intersections = new ArrayList<Float>();
    for(int i = 0; i < count; i++) {
      index += direction;
      if(index < 0) index += n;
      else if(index >= n) index -= n;
      PVector next = points.get(index);
      float nextLineDistance = line.DistanceOf(next);
      println("Next distance: " + nextLineDistance);
      
      // Here we manage the situation where the point is exactly on the line. In this case, we keep the same state as before:
      boolean nextIsLeft = nextLineDistance > 0 ? true : nextLineDistance < 0 ? false : isLeft;
      
      // Compute startPointOutside:
      if(i == 0) {
        // Rotate (next - current) to point outside the shape:
        float nX, nY;
        if(direction == 1) { nX = next.y - current.y; nY = current.x - next.x; }
        else {               nX = current.y - next.y; nY = next.x - current.x; }   
        
        // Compute the dot product between n and (line.start - current):
        float dotN = nX * (line.start.x - current.x) + nY * (line.start.y - current.y);
        
        // If the dot product is strictly greater than 0, the point is outside the shape:
        startPointOutside = dotN > 0;
        
        // But if the dot product equals zero (this can happen if line.start == current), the point is on the line:
        currentSideUnknown = dotN == 0;
      }
      
      // If both points are on the same side of the line, there is no intersection:
      if(nextIsLeft == isLeft) {
        println("Line: " + index + " => No intersection !");
      }
      else if(!currentSideUnknown) {
        float distance = line.IntersectDistance(current, next);
        if(distance > 0 && distance < 1)
          intersections.add(distance);
          
        println("Line: " + index + " => Intersection: " + distance);
      }
      else
        println("Ignoring intersection (next: " + index + ")");
      
      if(currentSideUnknown && nextLineDistance != 0) {
        startPointOutside = direction == 1 && nextLineDistance > 0 || direction == -1 && nextLineDistance < 0;
        currentSideUnknown = false;
      }
      current = next;
      isLeft = nextIsLeft;
    }
    
    println("Start point: " + (startPointOutside ? "outside" : "inside") + " " + (currentSideUnknown ? "(Unknown)" : "(Known)"));
    intersectionsCount = intersections.size();
    line.UpdateState(intersections, startPointOutside);
  }
  
  // Return the index of a point with an angle greater or equal to the given point:
  //private int FindSectionDeprecated(PVector point) {
  //  float pointAngle = atan2(point.y - center.y, point.x - center.x);
  //  if(pointAngle < 0) pointAngle += TWO_PI;
    
  //  int section = 0;
  //  while(section < angles.size() && angles.get(section) < pointAngle)
  //    section++;
    
  //  return section == angles.size() ? 0 : section;
  //}
  
  // Return the index of a point with an angle greater or equal to the given point:
  private int FindSectionUpper(PVector point) {
    float pointAngle = atan2(point.y - center.y, point.x - center.x);
    if(pointAngle < 0) pointAngle += TWO_PI;
    
    int section = 0;
    while(section < angles.size() && angles.get(section) < pointAngle)
      section++;
    
    return section == angles.size() ? 0 : section;
  }
  
  // Return the index of a point with an angle less or equal to the given point:
  private int FindSectionLower(PVector point) {
    float pointAngle = atan2(point.y - center.y, point.x - center.x);
    if(pointAngle < 0) pointAngle += TWO_PI;
    
    int section = angles.size() - 1;
    while(section >= 0 && angles.get(section) > pointAngle)
      section--;
    
    return section == -1 ? angles.size() - 1 : section;
  }
  
  
}
