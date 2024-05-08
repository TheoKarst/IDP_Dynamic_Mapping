import org.ejml.data.*;
import org.ejml.simple.*;

// Look at: dbscan

ArrayList<Cluster> previousClusters, currentClusters;

Kmeans kmeans;
MovingEntity entities[] = new MovingEntity[1];
Lidar lidar;
boolean pause = false;
boolean clearBackground = true;

boolean showLidar = true;
boolean showPoints = true;
boolean showClusters = true;
boolean showObjects = true;

color[] defaultColors = new color[] {
  color(255, 0, 0),
  color(0, 255, 0),
  color(0, 0, 255),
  
  color(255, 255, 0),
  color(255, 0, 255),
  color(0, 255, 255),
  
  color(255, 127, 127),
  color(127, 255, 127),
  color(127, 127, 255),
  
  color(255, 255, 127),
  color(255, 127, 255),
  color(127, 255, 255)
};

color[] myColors;

void setup() {
  size(1000, 600);
  ellipseMode(CENTER);
  rectMode(CENTER);
  strokeWeight(1);
  
  // Create entities:
  for(int i = 0; i < entities.length; i++) {
    float speed = 0.6f; //random(0, 0.7f);    // pixels / frames
    float radius = random(20, 50);
    entities[i] = new MovingEntity(random(width), random(height), speed, radius, 8, 25);
  }
    
  // Create the LIDAR:
  int raycastCount = 200;
  float speed = 0;
  float radius = 5;
  float startX = width / 4;
  float startY = height / 2;
  lidar = new Lidar(raycastCount, startX, startY, speed, radius, 8, 25);
  
  // Create colors for clustering:
  myColors = new color[raycastCount];
  for(int i = 0; i < myColors.length; i++) {
    myColors[i] = i < defaultColors.length ? defaultColors[i] : color(random(255), random(255), random(255));
  }
  
  if(pause)
    noLoop();
  
  // Run benchmarks for testing:
  // benchmarks();
}

void draw() {
  if(clearBackground) background(255);
  
  // Update and draw the entities:
  for(MovingEntity entity : entities)
    entity.Update();
  
  if(showObjects)
    for(MovingEntity entity : entities) entity.Draw();
  
  // Update and draw the LIDAR:
  lidar.Update();
  if(showLidar) lidar.Draw();
  
  // Get the hit points from the LIDAR, merge them by distance into clusters:
  PVector[] points = lidar.getHitPoints();
  ArrayList<ArrayList<PVector>> pointsClusters = myCreateClusters(points, 20);
  
  if(showPoints) {
    stroke(0);
    for(int i = 0; i < pointsClusters.size(); i++) {
      fill(myColors[i]);
      
      for(PVector point : pointsClusters.get(i))
        ellipse(point.x, point.y, 10, 10);
    }
  }
  
  // Create clusters:
  previousClusters = currentClusters;
  currentClusters = new ArrayList<Cluster>();
  for(int i = 0; i < pointsClusters.size(); i++) {
    Cluster cluster = new Cluster(pointsClusters.get(i), myColors[i]);
    cluster.build();
    currentClusters.add(cluster);
  }
  
  // Match previous clusters with the current ones to estimate their speed:
  // if(previousClusters != null)
  //   matchClusters();
  
  // Draw clusters:
  if(showClusters)
    for(Cluster cluster : currentClusters)
      cluster.Draw();
      
  // Draw the kmeans algorithm:
  if(kmeans == null) {
    kmeans = new Kmeans(20);
    kmeans.setupClusters(currentClusters);
  }
  
  kmeans.UpdateClusters(points);
  kmeans.Draw();
}

// Match previous clusters with the current ones:
void matchClusters() {
  ArrayList<Cluster> matches[] = new ArrayList[currentClusters.size()];
  for(int i = 0; i < matches.length; i++) matches[i] = new ArrayList<Cluster>();
  
  for(Cluster previous : previousClusters) {
    int match = -1;
    float minDistance = -1;
    
    for(int i = 0; i < currentClusters.size(); i++) {
      float distance = previous.distanceTo(currentClusters.get(i));
      
      if(match == -1 || distance < minDistance) {
        match = i;
        minDistance = distance;
      }
    }
    
    // Update speed estimate of previous:
    // float dX = currentClusters.get(match).x - previous.x;
    // float dY = currentClusters.get(match).y - previous.y;
    
    // previous.updateSpeedEstimate(dX, dY);
    matches[match].add(previous);
  }
  
  /*
  for(int i = 0; i < matches.length; i++) {
    if(matches[i].size() == 1) {
      Cluster current = currentClusters.get(i);
      Cluster previous = matches[i].get(0);
      
      // Filter position, orientation, dimensions, speed...
      final float mem = 0.9f;
      
      current.x = mem * previous.x + (1-mem) * current.x;
      current.y = mem * previous.y + (1-mem) * current.y;
      current.clusterAngle = mem * previous.clusterAngle + (1-mem) * current.clusterAngle;
      
      current.clusterSizeX = mem * previous.clusterSizeX + (1-mem) * current.clusterSizeX;
      current.clusterSizeY = mem * previous.clusterSizeY + (1-mem) * current.clusterSizeY;
      
      current.speedX = mem * previous.speedX + (1-mem) * current.speedX;
      current.speedY = mem * previous.speedY + (1-mem) * current.speedY;
    }
  }*/
  
  ArrayList<Cluster> toRemove = new ArrayList<Cluster>();
  
  for(int i = 0; i < matches.length; i++) {  
    float avgSpeedX = 0, avgSpeedY = 0;
    
    int totalSize = 0;
    if(matches[i].size() != 0) {
      for(Cluster cluster : matches[i]) {
        avgSpeedX += cluster.speedX * cluster.clusterSize();
        avgSpeedY += cluster.speedY * cluster.clusterSize();
        totalSize += cluster.clusterSize();
      }
      avgSpeedX /= totalSize;
      avgSpeedY /= totalSize;
    }
    
    currentClusters.get(i).updateSpeedEstimate(avgSpeedX, avgSpeedY);
    
    if(matches[i].size() > 1) {
      toRemove.add(currentClusters.get(i));
      
      for(Cluster cluster : matches[i]) {
        cluster.updateSpeedEstimate(avgSpeedX, avgSpeedY);
        cluster.clusterColor = color(currentClusters.get(i).clusterColor, 100);
        currentClusters.add(cluster);
      }
    }
  }
  
  for(Cluster c : toRemove)
    currentClusters.remove(c);
}

void keyPressed() {
  if(key == ' ') {
    pause = !pause;
    if(pause) noLoop();
    else {
      clearBackground = true;
      loop();
    }
  }
  
  if(pause && key == 'n') {  // Draw next frame without clearing
    clearBackground = false;
    redraw();
  }
  
  if(pause && key == 'r') {  // Clear and draw next frame
    clearBackground = true;
    redraw();
  }
  
  if(key == 'l') showLidar = !showLidar;
  if(key == 'p') showPoints = !showPoints;
  if(key == 'c') showClusters = !showClusters;
  if(key == 'o') showObjects = !showObjects;
}
  
