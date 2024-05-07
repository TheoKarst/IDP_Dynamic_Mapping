class MovingEntity {
  private final float speed;
  private final float radius;
  private final int minWaitMillis, maxWaitMillis;
  
  private float startX, startY;
  private float targetX, targetY;
  private float startTimeMillis;          // Time (in milliseconds) at which the entity was at the start position
  private float endTimeMillis;            // Predicted time at which the entity will reach the target
  private int targetWaitMillis;         // Duration the entity will wait at the target position before selecting a new target
  
  private float x, y;                     // Current position of the entity
  
  public MovingEntity(float speed, float radius, int minWaitMillis, int maxWaitMillis) {
    this.speed = speed;
    this.radius = radius;
    this.minWaitMillis = minWaitMillis;
    this.maxWaitMillis = maxWaitMillis;
    
    x = targetX = random(width);
    y = targetY = random(height);
    
    selectNewTarget();
  }
  
  public void Update() {
    int currentTime = millis();
    
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
  
  public void Draw() {
    stroke(0);
    fill(0, 0, 200, 100);
    
    ellipse(x, y, 2*radius, 2*radius);
  }
  
  public float intersectDistance(float originX, float originY, float dX, float dY) {
    // Vector from the entity origin to the raycast origin:
    float vX = originX - x;
    float vY = originY - y;
    
    float b = -dX*vX - dY*vY;
    float c = vX*vX + vY*vY - radius*radius;
    float delta = b*b - c;
    
    if(delta == 0)
      return b;
    else if(delta > 0) {
      float distance = b - sqrt(delta);
      
      if(distance >= 0)
        return distance;
    }
    
    return -1;    
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
