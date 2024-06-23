Shape shape;
Line line;

boolean drawShape = true;
int intersectionsCount = -1;

PVector lineStart, lineEnd;
int indexStart = -1, indexEnd = -1;

void setup() {
  size(1600, 1000, P2D);
  strokeWeight(2);
  ellipseMode(CENTER);
  textSize(32);
  
  shape = new Shape(new PVector(width/2, -height/2));
  shape.AddPoint(new PVector(width/2 + 300, -height/2 + 300));
  shape.AddPoint(new PVector(width/2 + 200, -height/2 + 300));
  shape.AddPoint(new PVector(width/2 + 100, -height/2 + 300));
  shape.AddPoint(new PVector(width/2 + 000, -height/2 + 300));
  shape.AddPoint(new PVector(width/2 - 100, -height/2 + 300));
  shape.AddPoint(new PVector(width/2 - 200, -height/2 + 300));
  
  shape.AddPoint(new PVector(width/2 - 300, -height/2 + 300));
  
  shape.AddPoint(new PVector(width/2 - 300, -height/2 + 200));
  shape.AddPoint(new PVector(width/2 - 300, -height/2 + 100));
  shape.AddPoint(new PVector(width/2 - 300, -height/2 + 000));
  shape.AddPoint(new PVector(width/2 - 300, -height/2 - 100));
  shape.AddPoint(new PVector(width/2 - 300, -height/2 - 200));
  shape.AddPoint(new PVector(width/2 - 300, -height/2 - 300));  
}

void draw() {
  background(255);
  
  fill(0);
  if(drawShape) {
    text("Draw shape", 10, 40);
  }
  else {
    text("Draw line", 10, 40);
  }
  
  if(intersectionsCount != -1)
    text("Intersections: " + intersectionsCount, 10, 80);
  
  shape.Draw();
  
  noFill();
  stroke(0, 255, 0);
  if(indexStart != -1) {
    PVector center = shape.points.get(indexStart);
    ellipse(center.x, -center.y, 10, 10);
  }
  if(indexEnd != -1) {
    PVector center = shape.points.get(indexEnd);
    ellipse(center.x, -center.y, 10, 10);
  }
  
  if(line != null)
    line.Draw();
    
  else if(lineStart != null && lineEnd != null) {
    stroke(0);
    line(lineStart.x, -lineStart.y, lineEnd.x, -lineEnd.y);
  }
}

void keyPressed() {
  if(key == 'c') {
    shape.Reset();
    drawShape = true;
    indexStart = indexEnd = -1;
    lineStart = lineEnd = null;
    intersectionsCount = -1;
  }
  
  if(keyCode == ENTER) {
    drawShape = false;
  }
    
  if(key == 'r') {
    lineStart = lineEnd = null;
    
    indexStart = randomPointIndex();
    indexEnd = randomPointIndex();
    lineStart = indexStart == -1 ? new PVector(random(width), -random(height)) : shape.points.get(indexStart);
    lineEnd = indexEnd == -1 ? new PVector(random(width), -random(height)) : shape.points.get(indexEnd);
    
    line = new Line(lineStart, lineEnd);
    shape.ComputeIntersectionsCorrected(line);
  }
}

void mousePressed() {
  if(drawShape) {
    shape.AddPoint(new PVector(mouseX, -mouseY));
  }
  else {
    indexStart = nearestPoint(shape);
    if(indexStart == -1) {
      lineStart = new PVector(mouseX, -mouseY);
    }
    else {
      lineStart = new PVector().set(shape.points.get(indexStart));
    }
    indexEnd = -1;
    lineEnd = null;
  }
}

void mouseReleased() {
  if(!drawShape) {
    indexEnd = nearestPoint(shape);
    if(indexEnd == -1) {
      lineEnd = new PVector(mouseX, -mouseY);
    }
    else {
      lineEnd = new PVector().set(shape.points.get(indexEnd));
    }
    
    line = new Line(lineStart, lineEnd);
    shape.ComputeIntersectionsCorrected(line);
  }
}

int randomPointIndex() {
  if(random(1) < 0.5f)
    return int(random(shape.points.size()));
    
  return -1;
}

int nearestPoint(Shape shape) {
  int nearestPoint = -1;
  float minDistance = 0;
  for(int i = 0; i < shape.points.size(); i++) {
    PVector point = shape.points.get(i);
    float dX = point.x - mouseX;
    float dY = point.y + mouseY;
    float distance = dX*dX + dY*dY;
    
    if(nearestPoint == -1 || distance < minDistance) {
      nearestPoint = i;
      minDistance = distance;
    }      
  }
  
  return minDistance < 50*50 ? nearestPoint : -1;
}
  
