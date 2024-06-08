Shape shape;
Line line;

void setup() {
  size(1600, 1000, P2D);
  strokeWeight(2);
  ellipseMode(CENTER);
  
  shape = new Shape(new PVector(width/2, -height/2));
  line = new Line(new PVector(width/4, -3*height/4), new PVector(3*width/4, -height/4));
  shape.ComputeIntersections(line);
}

void draw() {
  background(255);
  shape.Draw();
  line.Draw();
}

void keyPressed() {
  if(key == 'c')
    shape.Reset();
    
  if(key == 'r') {
    line = new Line(new PVector(random(width), -random(height)), new PVector(random(width), -random(height)));
    shape.ComputeIntersections(line);
  }
}

void mousePressed() {
  shape.AddPoint(new PVector(mouseX, -mouseY));
}
