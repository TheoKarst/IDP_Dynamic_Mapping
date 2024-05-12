import org.ejml.data.*;
import org.ejml.simple.*;

// Global parameters:
final int POINT_CLUSTER_LIFETIME = 100;
final float RECT_CLUSTER_MARGIN = 10;

final int LIDAR_RAYCAST_COUNT = 500;
final float RAYCAST_DISTANCE = 2000;
final int ENTITIES_COUNT = 1;

ClusterManager manager;
MovingEntity entities[] = new MovingEntity[ENTITIES_COUNT];
Lidar lidar;

boolean pause = true;
boolean showPoints = true;
boolean showLidar = true;
boolean showEntities = false;
boolean showStaticClusters = false;
boolean showDynamicClusters = true;

color[] myColors, defaultColors = new color[] {
  color(255, 0, 0),     color(0, 255, 0),     color(0, 0, 255),
  color(255, 255, 0),   color(255, 0, 255),   color(0, 255, 255),
  color(255, 127, 127), color(127, 255, 127), color(127, 127, 255),
  color(255, 255, 127), color(255, 127, 255), color(127, 255, 255)
};

int framesToDraw = -1;  // When pause == true, if we are looping, this is the number of frames before calling noLoop()

void setup() {
  size(2000, 1200, P2D);
  ellipseMode(CENTER);
  rectMode(CENTER);
  strokeWeight(1);
  
  // Create entities:
  float startX = random(width);
  float startY = random(height);
  float speed = 0;                          // pixels / frames
  float radius = 10;
  
  for(int i = 0; i < entities.length; i++) {
    startX = random(width);
    startY = random(height);
    speed = 1.2f;
    radius = random(60, 100);
    color shapeColor = color(0, 0, 255, 120);
    // Shape shape = new CircleShape(random(width), random(height), radius, shapeColor);
    Shape shape = new RectShape(startX, startY, 0, 200, 100, shapeColor);
    entities[i] = new MovingEntity(shape, speed);
  }
    
  // Create the LIDAR:
  startX = width / 4;
  startY = height / 2;
  speed = 0;
  radius = 5;  
  lidar = new Lidar(LIDAR_RAYCAST_COUNT, RAYCAST_DISTANCE, startX, startY, speed, radius);
  
  // Create colors for clustering:
  myColors = new color[LIDAR_RAYCAST_COUNT];
  for(int i = 0; i < myColors.length; i++) {
    myColors[i] = i < defaultColors.length ? defaultColors[i] : color(random(255), random(255), random(255));
  }
  
  manager = new ClusterManager(myColors, POINT_CLUSTER_LIFETIME, RECT_CLUSTER_MARGIN);
  
  if(pause)
    noLoop();
}

void draw() {
  background(255);
  
  // Testing:
  // final float mem = 0.9f;
  // lidar.x = mem * lidar.x + (1-mem) * mouseX;
  // lidar.y = mem * lidar.y + (1-mem) * mouseY;
  
  // Update and draw the entities:
  for(MovingEntity entity : entities)
    entity.Update();
  
  if(showEntities)
    for(MovingEntity entity : entities) entity.Draw();
  
  // Update and draw the LIDAR:
  lidar.Update();
  if(showLidar) lidar.Draw();
  
  // Get and draw the hit points from the LIDAR:
  PVector[] points = lidar.getHitPoints();
  
  if(showPoints) {
    fill(0); noStroke();
    for(PVector point : points)
      ellipse(point.x, point.y, 5, 5);
  }
  
  // Use these points to update the whole clustering algorithm:
  manager.UpdateClusters(points);
  
  // Draw the result of the clustering algorithm:
  manager.Draw(showStaticClusters, showDynamicClusters);
  
  if(pause && --framesToDraw <= 0)
    noLoop();
}


void keyPressed() {
  if(key == ' ') {
    pause = !pause;
    if(pause) noLoop();
    else {
      loop();
    }
  }
  
  if(pause && key == 'n') {  // Draw next frame
    framesToDraw = 10;
    loop();
  }
  
  if(key == 'p') showPoints = !showPoints;
  if(key == 'l') showLidar = !showLidar;
  if(key == 'e') showEntities = !showEntities;
  if(key == 's') showStaticClusters = !showStaticClusters;
  if(key == 'd') showDynamicClusters = !showDynamicClusters;
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
