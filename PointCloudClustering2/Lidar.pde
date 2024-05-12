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
    Raycast raycast = new Raycast(position(), new PVector(1, 0));
    for(int i = 0; i < raycastHits.length; i++) {
      raycastHits[i] = computeRaycastHit(raycast, raycastDistance);
      raycast.direction.rotate(TWO_PI / raycastHits.length);
    }
  }
  
  public void Draw() {
    super.Draw();
    
    stroke(255, 0, 0);
    PVector position = position();
    for(int i = 0; i < raycastHits.length; i++) {
      line(position.x, position.y, raycastHits[i].position.x, raycastHits[i].position.y);
    }
  }
  
  // Return if the given point is currently visible from the LIDAR:
  public boolean isVisible(PVector point, float margin) {
    PVector position = position();
    float angle = atan2(point.y - position.y, point.x - position.x);    // Angle between the given point and the lidar
    if(angle < 0) angle += TWO_PI;
    
    int prevIndex = floor(raycastHits.length * angle / TWO_PI);
    if(prevIndex == raycastHits.length) prevIndex--;
    
    int nextIndex = prevIndex + 1 < raycastHits.length ? prevIndex + 1 : 0;
    
    PVector prev = raycastHits[prevIndex].position;
    PVector next = raycastHits[nextIndex].position;
    
    // Create the vector normal to the line segment:
    PVector n = new PVector(next.y - prev.y, prev.x - next.x).normalize();
    
    return PVector.sub(point, prev, new PVector()).dot(n) <= -margin;
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
        positions[posIndex] = hit.position;
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
  public final PVector position;
  public final float hitDistance;
  public final boolean valid;        // If a hit was found or not
  
  public RaycastHit(PVector position, float hitDistance, boolean valid) {
    this.position = position;
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
  
  return new RaycastHit(intersect, maxDistance, hit);
}

class Raycast {
  public final PVector origin;
  public final PVector direction;
  
  public Raycast(PVector origin, PVector direction) {
    this.origin = origin;
    this.direction = direction.normalize();
  }
}
