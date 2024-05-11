import org.ejml.data.*;
import org.ejml.simple.*;

// Global parameters:
final int POINT_CLUSTER_LIFETIME = 100;
final float RECT_CLUSTER_MARGIN = 10;

final int LIDAR_RAYCAST_COUNT = 500;
final float RAYCAST_DISTANCE = 1000;
final int ENTITIES_COUNT = 5;

ClusterManager manager;
MovingEntity entities[] = new MovingEntity[ENTITIES_COUNT];
Lidar lidar;

boolean pause = true;
boolean showPoints = true;
boolean showLidar = false;
boolean showEntities = false;
boolean showStaticClusters = false;
boolean showDynamicClusters = true;

color[] myColors, defaultColors = new color[] {
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
  float speed = 0;                          // pixels / frames
  float radius = 10;
  
  for(int i = 0; i < entities.length; i++) {
    speed = 1.2f;
    radius = random(60, 100);
    entities[i] = new MovingEntity(random(width), random(height), speed, radius, 8, 25);
  }
    
  // Create the LIDAR:
  speed = 0;
  radius = 5;
  float startX = width / 4;
  float startY = height / 2;
  lidar = new Lidar(LIDAR_RAYCAST_COUNT, RAYCAST_DISTANCE, startX, startY, speed, radius, 8, 25);
  
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
    redraw();
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
