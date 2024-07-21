import org.ejml.data.*;
import org.ejml.simple.*;

final int LIDAR_RAYCAST_COUNT = 500;
final float RAYCAST_DISTANCE = 2000;
final int ENTITIES_COUNT = 10;

MovingEntity entities[] = new MovingEntity[ENTITIES_COUNT];
Lidar lidar;

boolean pause = false;
boolean followMouse = false;

boolean showPoints = true;
boolean showLidar = true;
boolean showEntities = true;

color[] defaultColors = new color[] {
  color(255, 0, 0),     color(0, 255, 0),     color(0, 0, 255),
  color(255, 255, 0),   color(255, 0, 255),   color(0, 255, 255),
  color(255, 127, 127), color(127, 255, 127), color(127, 127, 255),
  color(255, 255, 127), color(255, 127, 255), color(127, 255, 255)
};

void setup() {
  size(2000, 1200, P2D);
  ellipseMode(CENTER);
  rectMode(CENTER);
  strokeWeight(1);
  
  // Create entities:  
  for(int i = 0; i < entities.length; i++)
    entities[i] = randomEntity();
    
  // Create the LIDAR:
  lidar = new Lidar(LIDAR_RAYCAST_COUNT, RAYCAST_DISTANCE, width/2, height/2, 0, 5);
  
  if(pause)
    noLoop();
}

void draw() {
  background(255);
  
  // Testing:
  if(followMouse) {
    final float mem = 0.9f;
    
    lidar.shape.center.x = mem * lidar.shape.center.x + (1-mem) * mouseX;
    lidar.shape.center.y = mem * lidar.shape.center.y + (1-mem) * mouseY;
  }
  
  // Update and draw the entities:
  for(MovingEntity entity : entities)
    entity.Update();
  
  if(showEntities)
    for(MovingEntity entity : entities) entity.Draw();
  
  // Update and draw the LIDAR:
  lidar.Update();
  if(showLidar) lidar.Draw();
  
  // Get and draw the hit points from the LIDAR:
  Point[] points = lidar.getHitPoints();
  
  if(showPoints) {
    fill(0); noStroke();
    for(Point point : points)
      ellipse(point.x, point.y, 5, 5);
  }
  
  // Testing:
  ArrayList<Line> lines = lineExtraction(points);
  for(Line line : lines)
    line.Draw();
}

/* TEST:
Line line = new Line();
ArrayList<Point> testPoints = new ArrayList<Point>();
void draw() {
  background(255);
  stroke(0);
  fill(0, 0, 255);
  for(Point point : testPoints)
    ellipse(point.x, point.y, 5, 5);
    
  line.Draw();
  println("Distance: " + line.distanceFrom(new Point(mouseX, mouseY, 0)));
}

void mousePressed() {
  Point p = new Point(mouseX, mouseY, 0);
  testPoints.add(p);
  line.AddPoint(p);
  println("Line: (rho, theta) = (" + line.rho + ", " + degrees(line.theta) + ")");
}

void keyPressed() {
  testPoints.clear();
  line = new Line();
}*/

void keyPressed() {
  if(key == ' ') {
    pause = !pause;
    if(pause) noLoop();
    else {
      loop();
    }
  }
  
  if(key == 'f') followMouse = !followMouse;
  if(key == 'p') showPoints = !showPoints;
  if(key == 'l') showLidar = !showLidar;
  if(key == 'e') showEntities = !showEntities;
}

void drawArrow(float x1, float y1, float x2, float y2, color arrowColor) {
  PVector u = new PVector(x2 -x1, y2 - y1);
  float arrowLength = u.mag();
  
  if(arrowLength == 0)
    return;
  
  float triangleSize = min(40, arrowLength / 3);
  u.normalize().mult(triangleSize);
  PVector v = new PVector(u.y / 2, -u.x / 2);
  
  fill(arrowColor);
  stroke(arrowColor);
  line(x1, y1, x2, y2);
  triangle(x2, y2, x2 - u.x + v.x, y2 - u.y + v.y, x2 - u.x - v.x, y2 - u.y - v.y);
}

color randomColor() {
  return defaultColors[int(random(defaultColors.length))];
}

MovingEntity randomEntity() {
  color shapeColor = color(0, 0, 255, 120);
  
  float startX = random(width);
  float startY = random(height);
  float startAngle = random(TWO_PI);
  
  float radius = random(60, 100);
  
  float linearSpeed = 1.2f;
  float rotationSpeed = random(0.01f);
  
  Shape shape;
  if(random(1) < 0.1f) {
    shape = new CircleShape(startX, startY, radius, shapeColor);
  }
  else {
    int faceCount = int(random(3, 6));
    shape = new PolygonShape(startX, startY, startAngle, 2*radius, 2*radius, faceCount, shapeColor);
  }
  
  return new MovingEntity(shape, linearSpeed, rotationSpeed);
}
