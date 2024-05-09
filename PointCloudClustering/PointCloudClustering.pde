import org.ejml.data.*;
import org.ejml.simple.*;

// Global parameters:
final int MIN_POINTS_PER_CLUSTER = 3;
final float POINT_CLUSTER_VISIBILITY = 30;
final int POINT_CLUSTER_LIFETIME = 100;
final float RECT_CLUSTER_MARGIN = 10;

final int LIDAR_RAYCAST_COUNT = 500;
final int ENTITIES_COUNT = 5;

// Look at: dbscan
ClusterManager manager;
MovingEntity entities[] = new MovingEntity[ENTITIES_COUNT];
Lidar lidar;

boolean pause = false;

boolean showLidar = false;
boolean showRectClusters = true;
boolean showPointClusters = true;
boolean showEntities = true;

color[] defaultColors = new color[] {
  color(255, 0, 0),     color(0, 255, 0),     color(0, 0, 255),
  color(255, 255, 0),   color(255, 0, 255),   color(0, 255, 255),
  color(255, 127, 127), color(127, 255, 127), color(127, 127, 255),
  color(255, 255, 127), color(255, 127, 255), color(127, 255, 255)
};

void setup() {
  size(1000, 600);
  ellipseMode(CENTER);
  rectMode(CENTER);
  strokeWeight(1);
  
  // Create entities:
  for(int i = 0; i < entities.length; i++) {
    float speed = 0.6f; //random(0, 0.7f);    // pixels / frames
    float radius = random(30, 50);
    entities[i] = new MovingEntity(random(width), random(height), speed, radius, 8, 25);
  }
    
  // Create the LIDAR:
  float speed = 0;
  float radius = 5;
  float startX = width / 4;
  float startY = height / 2;
  lidar = new Lidar(LIDAR_RAYCAST_COUNT, startX, startY, speed, radius, 8, 25);
  
  // Create colors for clustering:
  color myColors[] = new color[LIDAR_RAYCAST_COUNT];
  for(int i = 0; i < myColors.length; i++) {
    myColors[i] = i < defaultColors.length ? defaultColors[i] : color(random(255), random(255), random(255));
  }
  
  manager = new ClusterManager(myColors, MIN_POINTS_PER_CLUSTER, POINT_CLUSTER_VISIBILITY, POINT_CLUSTER_LIFETIME, RECT_CLUSTER_MARGIN);
  
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
  
  // Get the hit points from the LIDAR, and use these points to run the whole clustering algorithm:
  manager.UpdateClusters(lidar.getHitPoints());
  
  // Draw the result of the clustering algorithm:
  manager.Draw(true, showRectClusters, showPointClusters);
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
    frameCount += 10;        // Use this to do bigger timesteps
    redraw();
  }
  
  if(key == 'l') showLidar = !showLidar;
  if(key == 'p') showPointClusters = !showPointClusters;
  if(key == 'r') showRectClusters = !showRectClusters;
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
