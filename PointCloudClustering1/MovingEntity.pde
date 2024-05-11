class MovingEntity {
  private final int minWaitFrames, maxWaitFrames;
  
  private float startX, startY;
  private float targetX, targetY;
  private float startTimeFrame;          // Frame at which the entity was at the start position
  private float endTimeFrame;            // Predicted frame at which the entity will reach the target
  private int targetWaitFrames;          // Duration the entity will wait at the target position before selecting a new target
  
  public final float speed;
  public final float radius;
  protected float x, y;                    // Current position of the entity
  
  public MovingEntity(float startX, float startY, float speed, float radius, int minWaitFrames, int maxWaitFrames) {
    this.speed = speed;
    this.radius = radius;
    this.minWaitFrames = minWaitFrames;
    this.maxWaitFrames = maxWaitFrames;
    
    x = targetX = startX;
    y = targetY = startY;
    
    selectNewTarget();
  }
  
  public void Update() {    
    if(frameCount < endTimeFrame) {
      x = map(frameCount, startTimeFrame, endTimeFrame, startX, targetX);
      y = map(frameCount, startTimeFrame, endTimeFrame, startY, targetY);
    }
    else if(frameCount < endTimeFrame + targetWaitFrames) {
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
    targetWaitFrames = (int) random(minWaitFrames, maxWaitFrames);
    
    float dX = startX - targetX;
    float dY = startY - targetY;
    float distance = sqrt(dX*dX + dY*dY);
    
    startTimeFrame = frameCount;
    endTimeFrame = startTimeFrame + distance / speed;
  }
}
