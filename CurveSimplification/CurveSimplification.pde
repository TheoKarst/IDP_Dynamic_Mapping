ArrayList<PVector> initialCurve = new ArrayList<PVector>();
ArrayList<PVector> transformedCurve = new ArrayList<PVector>();

void setup() {
  size(1600, 1000, P2D);
  
}

void draw() {
  background(255);
  drawCurve(initialCurve, color(0, 0, 255), false);
  drawCurve(transformedCurve, color(0, 255, 0), true);
}

void mousePressed() {
  transformedCurve.clear();
  initialCurve.add(new PVector(mouseX, mouseY));
}

void keyPressed() {
  if(key == 'a') {
    println("Douglas Peucker");
    println("IMPLEMENT THIS !");
  }
  else if(key == 'z') {
    println("Personal implementation");
    transformedCurve = curveRegularization(initialCurve, 100);
  }
  else if(key == 'c') {
    initialCurve.clear();
    transformedCurve.clear();
  }
}

void drawCurve(ArrayList<PVector> points, color curveColor, boolean drawPoints) {
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
  
  if(drawPoints) {
    noStroke();
    fill(0);
    for(PVector point : points)
      ellipse(point.x, point.y, 5, 5);
  }
}
