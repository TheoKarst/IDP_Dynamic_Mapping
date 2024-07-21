ArrayList<PVector> transformedPoints = new ArrayList<PVector>();
boolean match[] = null;

Curve initialCurve = new Curve(new ArrayList<PVector>(), 20, color(0, 255, 0));

enum DrawMode {
  INITIAL_SHAPE,
  TRANSFORMED_SHAPE,
  DONE
}

DrawMode drawMode = DrawMode.INITIAL_SHAPE;

void setup() {
  size(2000, 1200, P2D);
  ellipseMode(CENTER);
  strokeWeight(2);
  textSize(32);
}

void draw() {
  background(255);
  
  initialCurve.Draw();
  drawCurve(transformedPoints, color(0, 0, 255));
  
  if(match != null) {
    stroke(0);
    
    for(int i = 0; i < match.length; i++) {
      fill(match[i] ? color(0, 255, 0) : color(255, 0, 0));
      ellipse(transformedPoints.get(i).x, transformedPoints.get(i).y, 20, 20);
    }
  }
  
  initialCurve.DrawContour();
  initialCurve.checkInside(new PVector(mouseX, mouseY));
  
  // Draw text info:
  fill(0);
  if(drawMode == DrawMode.INITIAL_SHAPE)
    text("Draw initial shape", 10, 40);
    
  else if(drawMode == DrawMode.TRANSFORMED_SHAPE)
    text("Draw transformed shape", 10, 40);
}

void mousePressed() {
  if(drawMode == DrawMode.INITIAL_SHAPE)
    initialCurve.points.add(new PVector(mouseX, mouseY));
    
  else if(drawMode == DrawMode.TRANSFORMED_SHAPE)
    transformedPoints.add(new PVector(mouseX, mouseY));
}

void keyPressed() {
  if(key == 'c') {
    initialCurve.points.clear();
    initialCurve.isLoop = false;
    
    transformedPoints.clear();
    match = null;
    drawMode = DrawMode.INITIAL_SHAPE;
  }
  else if(key == 't' && drawMode == DrawMode.INITIAL_SHAPE && initialCurve.points.size() > 1) {
    drawMode = DrawMode.TRANSFORMED_SHAPE;
  }
  else if(key == ENTER && drawMode == DrawMode.TRANSFORMED_SHAPE && transformedPoints.size() > 1) {
    drawMode = DrawMode.DONE;
    
    match = initialCurve.MatchCurve(transformedPoints);
  }
  else if(key == 'r' && drawMode == DrawMode.DONE) {
    initialCurve.points = curveRegularization(initialCurve.points, 100);
  }
}

void drawCurve(ArrayList<PVector> points, color curveColor) {
  if(points.size() < 2)
    return;
      
  strokeWeight(2);
  stroke(curveColor);
  PVector previous = points.get(0);
  for(int i = 1; i < points.size(); i++) {
    PVector current = points.get(i);
    line(previous.x, previous.y, current.x, current.y);
    previous = current;
  }
}
