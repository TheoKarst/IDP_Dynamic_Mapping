class Lidar extends MovingEntity {  
  private final float raycastDistance;
  private RaycastHit[] raycastHits;
  
  public Lidar(int raycastCount, float raycastDistance, float startX, float startY, float speed, float radius) {
    super(new CircleShape(startX, startY, radius, color(200, 0, 0)), speed);
    
    this.raycastDistance = raycastDistance;
    this.raycastHits = new RaycastHit[raycastCount];
  }
  
  public void Update() {
    super.Update();
      
    // Update the raycast intersections:
    Raycast raycast = new Raycast(x, y, new PVector(1, 0));
    for(int i = 0; i < raycastHits.length; i++) {
      raycastHits[i] = computeRaycastHit(raycast, raycastDistance);
      raycast.direction.rotate(TWO_PI / raycastHits.length);
    }
  }
  
  public void Draw() {
    super.Draw();
    
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

RaycastHit computeRaycastHit(Raycast raycast, float maxDistance) {
  
  boolean hit = false;
  for(MovingEntity entity : entities) {
    float distance = entity.intersectDistance(raycast);
    
    if(distance >= 0 && distance < maxDistance) {
      maxDistance = distance;
      hit = true;
    }
  }
    
  PVector intersect = new PVector().set(raycast.direction).mult(maxDistance).add(raycast.origin);
  
  return new RaycastHit(intersect.x, intersect.y, maxDistance, hit);
}

class Raycast {
  public final PVector origin;
  public final PVector direction;
  
  public Raycast(float originX, float originY, PVector direction) {
    this.origin = new PVector(originX, originY);
    this.direction = direction.normalize();
  }
}
