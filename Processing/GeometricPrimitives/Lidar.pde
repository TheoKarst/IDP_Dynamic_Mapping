class Lidar extends MovingEntity {  
  private final float raycastDistance;
  private RaycastHit[] raycastHits;
  
  public Lidar(int raycastCount, float raycastDistance, float startX, float startY, float speed, float radius) {
    super(new CircleShape(startX, startY, radius, color(200, 0, 0)), speed, 0);
    
    this.raycastDistance = raycastDistance;
    this.raycastHits = new RaycastHit[raycastCount];
  }
  
  public void Update() {
    // super.Update();
      
    // Update the raycast intersections:
    Raycast raycast = new Raycast(position(), new PVector(1, 0));
    float raycastAngle = 0;
    float deltaAngle = TWO_PI / raycastHits.length;
    
    for(int i = 0; i < raycastHits.length; i++) {
      raycastHits[i] = computeRaycastHit(raycast, raycastAngle, raycastDistance);
      raycast.direction.rotate(deltaAngle);
      raycastAngle += deltaAngle;
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
  
  public Point[] getHitPoints() {
    int hitCount = 0;
    for(RaycastHit hit : raycastHits)
      if(hit.valid)
        hitCount++;
        
    Point[] points = new Point[hitCount];
    
    int posIndex = 0;
    for(RaycastHit hit : raycastHits){
      if(hit.valid) {
        points[posIndex] = hit.position;
        posIndex++;
      }
    }
      
    return points;
  }
  
  public float[] getHitDistances() {
    float[] distances = new float[raycastHits.length];
    
    for(int i = 0; i < distances.length; i++)    
      distances[i] = raycastHits[i].hitDistance;
      
    return distances;
  }      
}

class RaycastHit {
  public final Point position;
  public final float hitDistance;
  public final boolean valid;        // If a hit was found or not
  
  public RaycastHit(Point position, float hitDistance, boolean valid) {
    this.position = position;
    this.hitDistance = hitDistance;
    this.valid = valid;
  }
}

RaycastHit computeRaycastHit(Raycast raycast, float raycastAngle, float maxDistance) {
  
  boolean hit = false;
  for(MovingEntity entity : entities) {
    float distance = entity.intersectDistance(raycast);
    
    if(distance >= 0 && distance < maxDistance) {
      maxDistance = distance;
      hit = true;
    }
  }
    
  PVector intersect = new PVector().set(raycast.direction).mult(maxDistance).add(raycast.origin);
  
  return new RaycastHit(new Point(intersect.x, intersect.y, raycastAngle), maxDistance, hit);
}

class Raycast {
  public final PVector origin;
  public final PVector direction;
  
  public Raycast(PVector origin, PVector direction) {
    this.origin = origin;
    this.direction = direction.normalize();
  }
}
