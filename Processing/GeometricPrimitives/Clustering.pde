final float POINT_CRITICAL_DISTANCE = 20;
final float LINE_CRITICAL_DISTANCE = 4;
final float CRITICAL_ALPHA = radians(10);

static class Point {
  public final float x;
  public final float y;
  public final float angle;
  
  public Point(float x, float y, float angle) {
    this.x = x;
    this.y = y;
    this.angle = angle;
  }
  
  public static float dist(Point a, Point b) {
    float dX = a.x - b.x;
    float dY = a.y - b.y;
    
    return sqrt(dX*dX + dY*dY);
  }
  
  public static float angularDifference(Point a, Point b) {
    return abs(a.angle - b.angle);
  }
}

class Line {
  
  // Points representing this line:
  private ArrayList<Point> points = new ArrayList<Point>();
  
  // Regression parameters of the line:
  private float Rx = 0, Ry = 0, Rxx = 0, Ryy = 0, Rxy = 0;
  
  // Parameters defining the line:
  private float rho, theta;
  
  // Endpoints of the line:
  private PVector beginPoint, endPoint;
  
  public void Draw() {
    stroke(0, 255, 0);
    strokeWeight(4);
    line(beginPoint.x, beginPoint.y, endPoint.x, endPoint.y);
    strokeWeight(1);
  }
  
  // Add a point to the line, and update the line parameters:
  public void AddPoint(Point point) {
    points.add(point);
    
    Rx += point.x;
    Ry += point.y;
    Rxx += point.x * point.x;
    Ryy += point.y * point.y;
    Rxy += point.x * point.y;
    
    int n = points.size();
    float N1 = Rxx*n - Rx*Rx;
    float N2 = Ryy*n - Ry*Ry;
    float T = Rxy*n - Rx*Ry;
    
    // N1 and N2 represent the width of the cloud of the regression points along the X- and Y-axis. If N1 is larger 
    // than N2, the cloud of points lies more horizontally than vertically which makes regression of y to x (y = mx + q)
    // more favourable. Otherwise, the regression of x to y is selected (x = sy + t):
    if(N1 >= N2) {
      float m = T / N1;
      float q = (Ry - m*Rx) / n;
      
      rho = abs(q / sqrt(m*m + 1));
      theta = atan2(q, -q*m);
    }
    else {
      float s = T / N2;
      float t = (Rx - s*Ry) / n;
      
      rho = abs(t / sqrt(s*s + 1));
      theta = atan2(-t*s, t);
    }
  }
  
  // Return the distance between the line and the given point:
  float distanceFrom(Point point) {
    return abs(point.x * cos(theta) + point.y * sin(theta) - rho);
  }
  
  Point getLastPoint() {
    int n = points.size();
    return n == 0 ? null : points.get(n-1);
  }
  
  int pointsCount() {
    return points.size();
  }
  
  void computeEndpoints() {
    float x = rho * cos(theta);
    float y = rho * sin(theta);
    
    PVector u = new PVector(0, 1);
    u.rotate(theta);
    
    // Compute the projection of the first point and last point of the line:
    Point firstPoint = points.get(0);
    Point lastPoint = points.get(points.size()-1);
    
    float pFirst = u.x * (firstPoint.x - x) + u.y * (firstPoint.y - y);
    float pLast  = u.x * (lastPoint.x - x) + u.y * (lastPoint.y - y);
    
    beginPoint = new PVector(x + pFirst * u.x, y + pFirst * u.y);
    endPoint = new PVector(x + pLast * u.x, y + pLast * u.y);
  }
}


// Line extraction, following section 4.3.1:
ArrayList<Line> lineExtraction(Point[] points) {
  ArrayList<Line> extractedLines = new ArrayList<Line>();
  Line currentLine = new Line();
  
  for(Point point : points) {
    
    // Get the last point that was added to the line:
    Point lastPoint = currentLine.getLastPoint();
    
    if(lastPoint == null) {
      currentLine.AddPoint(point);
    }
    else {
      boolean condition1 = Point.dist(lastPoint, point) <= POINT_CRITICAL_DISTANCE;
      boolean condition2 = currentLine.pointsCount() < 3 || currentLine.distanceFrom(point) <= LINE_CRITICAL_DISTANCE;
      boolean condition3 = Point.angularDifference(lastPoint, point) <= CRITICAL_ALPHA;
      
      // If the three conditions are met, we can add the point to the line. Otherwise, the extraction is aborted
      // and the parameters of the line are calculated:
      if(condition1 && condition2 && condition3) {
        currentLine.AddPoint(point);
      }
      else {
        if(currentLine.pointsCount() >= 3) {
          currentLine.computeEndpoints();
          extractedLines.add(currentLine);
        }
        
        currentLine = new Line();
        currentLine.AddPoint(point);
      }
    }
  }
  
  if(currentLine.pointsCount() >= 3) {
    currentLine.computeEndpoints();
    extractedLines.add(currentLine);
  }
  
  return extractedLines;
}
