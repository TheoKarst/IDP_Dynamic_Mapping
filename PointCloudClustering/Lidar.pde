class Lidar extends MovingEntity {  
  private RaycastHit[] raycastHits;
  
  public Lidar(int raycastCount, float startX, float startY, float speed, float radius, int minWaitFrames, int maxWaitFrames) {
    super(startX, startY, speed, radius, minWaitFrames, maxWaitFrames);
    
    raycastHits = new RaycastHit[raycastCount];
  }
  
  public void Update() {
    super.Update();
      
    // Update the raycast intersections:
    PVector d = new PVector(1, 0);
    for(int i = 0; i < raycastHits.length; i++) {
      raycastHits[i] = computeRaycastHit(x, y, d, 1000);
      d.rotate(TWO_PI / raycastHits.length);
    }
  }
  
  public void Draw() {
    stroke(0);
    fill(200, 0, 0);
    
    ellipse(x, y, 2*radius, 2*radius);
    
    stroke(255, 0, 0);
    for(RaycastHit hit : raycastHits) {
      line(x, y, hit.x, hit.y);
    }
  }
  
  public PVector[] getHitPoints() {
    int hitCount = 0;
    for(RaycastHit hit : raycastHits)
      if(hit.valid)
        hitCount++;
        
    PVector[] positions = new PVector[hitCount];
    
    int posIndex = 0;
    for(RaycastHit hit : raycastHits){
      if(hit.valid) {
        positions[posIndex] = new PVector(hit.x, hit.y);
        posIndex++;
      }
    }
      
    return positions;
  }
}

class RaycastHit {
  public final float x, y;
  public final boolean valid;  // If a hit was found or not
  
  public RaycastHit(float x, float y, boolean valid) {
    this.x = x;
    this.y = y;
    this.valid = valid;
  }
}

RaycastHit computeRaycastHit(float originX, float originY, PVector raycastDirection, float maxDistance) {
  raycastDirection.normalize();
  
  boolean hit = false;
  for(MovingEntity entity : entities) {
    float distance = entity.intersectDistance(originX, originY, raycastDirection.x, raycastDirection.y);
    
    if(distance >= 0 && distance < maxDistance) {
      maxDistance = distance;
      hit = true;
    }
  }
    
  float intersectX = originX + raycastDirection.x * maxDistance;
  float intersectY = originY + raycastDirection.y * maxDistance;
  
  return new RaycastHit(intersectX, intersectY, hit);
}
