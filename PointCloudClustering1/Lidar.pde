class Lidar extends MovingEntity {  
  private final float raycastDistance;
  private RaycastHit[] raycastHits;
  
  public Lidar(int raycastCount, float raycastDistance, float startX, float startY, float speed, float radius, int minWaitFrames, int maxWaitFrames) {
    super(startX, startY, speed, radius, minWaitFrames, maxWaitFrames);
    
    this.raycastDistance = raycastDistance;
    this.raycastHits = new RaycastHit[raycastCount];
  }
  
  public void Update() {
    super.Update();
      
    // Update the raycast intersections:
    PVector d = new PVector(1, 0);
    for(int i = 0; i < raycastHits.length; i++) {
      raycastHits[i] = computeRaycastHit(x, y, d, raycastDistance);
      d.rotate(TWO_PI / raycastHits.length);
    }
  }
  
  public void Draw() {
    stroke(0);
    fill(200, 0, 0);
    
    ellipse(x, y, 2*radius, 2*radius);
    
    stroke(255, 0, 0);
    for(int i = 0; i < raycastHits.length; i++) {
      line(x, y, raycastHits[i].x, raycastHits[i].y);
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
  
  public float[] getHitDistances() {
    float[] distances = new float[raycastHits.length];
    
    for(int i = 0; i < distances.length; i++)    
      distances[i] = raycastHits[i].hitDistance;
      
    return distances;
  }      
}

class RaycastHit {
  public final float x, y;
  public final float hitDistance;
  public final boolean valid;        // If a hit was found or not
  
  public RaycastHit(float x, float y, float hitDistance, boolean valid) {
    this.x = x;
    this.y = y;
    this.hitDistance = hitDistance;
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
  
  return new RaycastHit(intersectX, intersectY, maxDistance, hit);
}
