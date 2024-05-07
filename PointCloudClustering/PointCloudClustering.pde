import org.ejml.data.*;
import org.ejml.simple.*;

MovingEntity entities[] = new MovingEntity[15];
Lidar lidar;
boolean pause = false;

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

class Point {
  public final float x, y;
  public final boolean hit;
  
  public Point(float x, float y, boolean hit) {
    this.x = x;
    this.y = y;
    this.hit = hit;
  }
}

color[] myColors;

void setup() {
  size(1000, 600);
  ellipseMode(CENTER);
  rectMode(CENTER);
  strokeWeight(1);
  
  // Create entities:
  for(int i = 0; i < entities.length; i++) {
    float speed = random(0, 40);
    float radius = random(20, 50);
    entities[i] = new MovingEntity(speed, radius, 500, 1500);
  }
    
  // Create the LIDAR:
  int raycastCount = 200;
  float speed = 0;
  float radius = 5;
  float startX = width / 4;
  float startY = height / 2;
  lidar = new Lidar(raycastCount, startX, startY, speed, radius, 500, 1500);
  
  // Create colors for clustering:
  myColors = new color[raycastCount];
  for(int i = 0; i < myColors.length; i++) {
    myColors[i] = i < defaultColors.length ? defaultColors[i] : color(random(255), random(255), random(255));
  }
  
  // Run benchmarks for testing:
  // benchmarks();
}

void draw() {
  background(255);
  
  // Update and draw the entities:
  for(MovingEntity entity : entities) {
    entity.Update();
    entity.Draw();
  }
  
  // Update and draw the LIDAR:
  lidar.Update();
  lidar.Draw();
  
  // Get the intersect points from the LIDAR, and label them using Fast Euclidian Clustering:
  PVector[] points = lidar.getIntersectPoints();
  
  int[] labels = myCreateClusters(points, 20); // createClusters(points, 50);
  
  stroke(0);
  int maxLabel = 0;
  for(int i = 0; i < points.length; i++) {
    fill(myColors[labels[i]-1]);
    ellipse(points[i].x, points[i].y, 10, 10);
    
    if(labels[i] > maxLabel)
      maxLabel = labels[i];
  }
  
  // Group points by label:
  ArrayList<PVector> groups[] = new ArrayList[maxLabel];
  for(int i = 0; i < maxLabel; i++) groups[i] = new ArrayList();
  
  for(int i = 0; i < points.length; i++)
    groups[labels[i]-1].add(points[i]);
  
  // Create clusters:
  Cluster clusters[] = new Cluster[maxLabel];
  for(int i = 0; i < maxLabel; i++) {
    clusters[i] = new Cluster(groups[i], myColors[i]);
    clusters[i].build();
    clusters[i].Draw();
  }
}

Point intersectPoint(float originX, float originY, PVector raycastDirection, float maxDistance) {
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
  
  return new Point(intersectX, intersectY, hit);
}

void benchmarks() {
  PVector[] points = generateRandomPoints(80000);
  
  // Create benchmarks for fast euclidian clustering:
  int t;
  t = millis(); createClusters(points, 0.1f); t = millis() - t;
  println("Fast Euclidian clustering: " + t / 1000f + "s");
  
  t = millis(); createClusters(points, 5); t = millis() - t;
  println("Fast Euclidian clustering: " + t / 1000f + "s");
  
  t = millis(); createClusters(points, 20); t = millis() - t;
  println("Fast Euclidian clustering: " + t / 1000f + "s");
  
  
  // Create benchmarks for my implementation:
  t = millis(); myCreateClusters(points, 0.1f); t = millis() - t;
  println("Personal clustering: " + t / 1000f + "s");
  
  t = millis(); myCreateClusters(points, 5); t = millis() - t;
  println("Personal clustering: " + t / 1000f + "s");
  
  t = millis(); myCreateClusters(points, 20); t = millis() - t;
  println("Personal clustering: " + t / 1000f + "s");
}

PVector[] generateRandomPoints(int count) {
  PVector[] points = new PVector[count];
  
  for(int i = 0; i < count; i++) {
    points[i] = new PVector(random(100), random(100));
  }
  
  return points;
}

void keyPressed() {
  if(key == ' ') {
    pause = !pause;
    if(pause) noLoop();
    else loop();
  }
}
  
