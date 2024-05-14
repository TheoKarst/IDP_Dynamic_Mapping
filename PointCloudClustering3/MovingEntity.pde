class MovingEntity {
  private final int minWaitFrames = 8;
  private final int maxWaitFrames = 25;
  
  public Shape shape;
  
  private final float speed;
  
  private float startX, startY;
  private float targetX, targetY;
  
  private float startTimeFrame;          // Frame at which the entity was at the start position
  private float endTimeFrame;            // Predicted frame at which the entity will reach the target
  private int targetWaitFrames;          // Duration the entity will wait at the target position before selecting a new target
  
  public MovingEntity(Shape shape, float speed) {
    this.speed = speed;
    this.shape = shape;
    
    targetX = startX = shape.center.x;
    targetY = startY = shape.center.y;
    
    selectNewTarget();
  }
  
  public void Update() {
    if(frameCount < endTimeFrame) {
      float x = map(frameCount, startTimeFrame, endTimeFrame, startX, targetX);
      float y = map(frameCount, startTimeFrame, endTimeFrame, startY, targetY);
      shape.Update(x, y, 0);
    }
    else if(frameCount < endTimeFrame + targetWaitFrames)
      shape.Update(targetX, targetY, 0);
    
    else
      selectNewTarget();
  }
  
  public void Draw() {
    shape.Draw();
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
  
  public float intersectDistance(Raycast raycast) {
    return shape.intersectDistance(raycast);
  }
  
  public PVector position() {
    return shape.center;
  }
}

abstract class Shape {
  public PVector center;
  public float angle;
  
  public Shape(float x, float y, float angle) {
    this.center = new PVector(x, y);
    this.angle = angle;
  }
  
  void Update(float x, float y, float angle) {
    center.set(x, y);
    this.angle = angle;
  }
  
  abstract void Draw();
  
  abstract float intersectDistance(Raycast raycast);
  
  PVector getPosition() {
    return center;
  }
}

class CircleShape extends Shape {
  private float radius;
  private color shapeColor;
  
  public CircleShape(float x, float y, float radius, color shapeColor) {
    super(x, y, 0);
    this.radius = radius;
    this.shapeColor = shapeColor;
  }
  
  void Update(float x, float y, float angle) {
    this.center.set(x, y);
  }
  
  void Draw() {
    stroke(0);
    fill(shapeColor);
    ellipse(center.x, center.y, 2*radius, 2*radius);
  }
  
  float intersectDistance(Raycast raycast) {
    // Vector from the entity origin to the raycast origin:
    PVector v = PVector.sub(raycast.origin, center, new PVector());
    
    float b = -raycast.direction.dot(v);
    float c = v.magSq() - radius*radius;
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
}

public class PolygonShape extends Shape {
  private final PVector[] initialPoints;
  private PVector[] transformedPoints;
  
  private color shapeColor;
  
  public PolygonShape(float x, float y, float angle, float Width, float Height, int pointsCount, color shapeColor) {
    super(x, y, angle);
    this.initialPoints = buildRegularPolygon(Width, Height, pointsCount);
    initTransformPoints(x, y, angle);
    this.shapeColor = shapeColor;
  }
  
  public PolygonShape(float x, float y, float angle, PVector[] points, color shapeColor) {
    super(x, y, angle);
    this.initialPoints = points;
    initTransformPoints(x, y, angle);
    this.shapeColor = shapeColor;
  }
  
  private void initTransformPoints(float x, float y, float angle) {
    // Create an array to store the transformed points:
    transformedPoints = new PVector[initialPoints.length];
    for(int i = 0; i < transformedPoints.length; i++) transformedPoints[i] = new PVector();
    
    // Apply the transformation:
    transformPoints(initialPoints, x, y, angle, transformedPoints);
  }
  
  void Update(float x, float y, float angle) {
    transformPoints(initialPoints, x, y, angle, transformedPoints);
  }
  
  void Draw() {
    stroke(shapeColor);
    
    PVector previous = transformedPoints[transformedPoints.length-1];
    for(PVector point : transformedPoints) {
      line(previous.x, previous.y, point.x, point.y);
      previous = point;
    }
  }
  
  float intersectDistance(Raycast raycast) {
    float minDistance = -1;
    
    PVector previous = transformedPoints[transformedPoints.length-1];
    for(PVector point : transformedPoints) {
      float distance = lineIntersect(raycast, previous, point);
      
      if(distance >= 0 && (minDistance < 0 || distance < minDistance))
        minDistance = distance;
        
      previous = point;
    }
    
    return minDistance;
  }
  
  private void transformPoints(PVector[] points, float x, float y, float angle, PVector[] target) {
    for(int i = 0; i < points.length; i++)
      target[i].set(points[i]).rotate(angle).add(x, y);
  }
  
  private PVector[] buildRegularPolygon(float Width, float Height, int pointsCount) {
    PVector[] points = new PVector[pointsCount];
    
    PVector current = new PVector(0, -1);
    if(pointsCount % 2 == 0) current.rotate(PI / pointsCount);
    
    for(int i = 0; i < pointsCount; i++) {
      points[i] = new PVector(current.x * Width / 2, current.y * Height / 2);
      current.rotate(TWO_PI / pointsCount);
    }
    
    return points;
  }
  
  private float lineIntersect(Raycast raycast, PVector lineStart, PVector lineEnd) {
    PVector AB = PVector.sub(lineEnd, lineStart, new PVector());
    PVector OA = PVector.sub(lineStart, raycast.origin, new PVector());
    PVector d = raycast.direction;
    
    float den = AB.x * d.y - AB.y * d.x;
    
    // No intersection (lines are parallel):
    if(den == 0)
      return -1;
      
    float x = (OA.y*d.x - OA.x*d.y) / den;
    
    // No intersection (outside line bounds):
    if(x < 0 || x > 1)
      return -1;
      
    return (OA.y * AB.x - OA.x * AB.y) / den;
  }
  
  float getX() { return center.x; }
  float getY() { return center.y; }
}
