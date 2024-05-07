class Lidar {  
  private final float speed;
  private final float radius;
  private final int minWaitMillis, maxWaitMillis;
  
  private float startX, startY;
  private float targetX, targetY;
  private float startTimeMillis;          // Time (in milliseconds) at which the entity was at the start position
  private float endTimeMillis;            // Predicted time at which the entity will reach the target
  private int targetWaitMillis;         // Duration the entity will wait at the target position before selecting a new target
  
  private float x, y;                     // Current position of the entity
  
  private Point[] intersectPoints;
  
  public Lidar(int raycastCount, float startX, float startY, float speed, float radius, int minWaitMillis, int maxWaitMillis) {
    intersectPoints = new Point[raycastCount];
    
    this.speed = speed;
    this.radius = radius;
    this.minWaitMillis = minWaitMillis;
    this.maxWaitMillis = maxWaitMillis;
    
    x = targetX = startX;
    y = targetY = startY;
    
    if(speed != 0)
      selectNewTarget();
  }
  
  public void Update() {
    if(speed != 0) {
      int currentTime = millis();
      
      // Update the LIDAR position:
      if(currentTime < endTimeMillis) {
        x = map(currentTime, startTimeMillis, endTimeMillis, startX, targetX);
        y = map(currentTime, startTimeMillis, endTimeMillis, startY, targetY);
      }
      else if(currentTime < endTimeMillis + targetWaitMillis) {
        x = targetX;
        y = targetY;
      }
      else
        selectNewTarget();
    }
      
    // Update the raycast intersections:
    PVector d = new PVector(1, 0);
    for(int i = 0; i < intersectPoints.length; i++) {
      intersectPoints[i] = intersectPoint(x, y, d, 400);
      d.rotate(TWO_PI / intersectPoints.length);
    }
  }
  
  public void Draw() {
    stroke(0);
    fill(200, 0, 0);
    
    ellipse(x, y, 2*radius, 2*radius);
    
    stroke(255, 0, 0);
    for(Point intersect : intersectPoints) {
      line(x, y, intersect.x, intersect.y);
    }
  }
  
  public PVector[] getIntersectPoints() {
    int hitCount = 0;
    for(Point point : intersectPoints)
      if(point.hit)
        hitCount++;
        
    PVector[] positions = new PVector[hitCount];
    
    int posIndex = 0;
    for(Point point : intersectPoints){
      if(point.hit) {
        positions[posIndex] = new PVector(point.x, point.y);
        posIndex++;
      }
    }
      
    return positions;
  }
  
  private void selectNewTarget() {
    startX = targetX;
    startY = targetY;
    
    targetX = random(width);
    targetY = random(height);
    targetWaitMillis = (int) random(minWaitMillis, maxWaitMillis);
    
    float dX = startX - targetX;
    float dY = startY - targetY;
    float distance = sqrt(dX*dX + dY*dY);
    
    startTimeMillis = millis();
    endTimeMillis = startTimeMillis + 1000 * distance / speed;
  }
}
