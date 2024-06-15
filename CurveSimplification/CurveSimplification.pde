ArrayList<PVector> initialCurve = new ArrayList<PVector>();
ArrayList<PVector> transformedCurve = new ArrayList<PVector>();

void setup() {
  size(1600, 1000, P2D);
  
  initialCurve = randomCurve(50000, new PVector(width/2, height/2), height/10, height/2);
  benchmarks();
}

void draw() {
  background(255);
  drawCurve(initialCurve, color(0, 0, 255), false, 1);
  drawCurve(transformedCurve, color(0, 255, 0), true, 1);
}

void mousePressed() {
  transformedCurve.clear();
  initialCurve.add(new PVector(mouseX, -mouseY));
}

void keyPressed() {
  if(key == 'a') {
    println("Douglas Peucker");
    transformedCurve = DouglasPeucker(initialCurve, 100);
  }
  else if(key == 'z') {
    print("Curve regularization 1: ");
    transformedCurve = curveRegularization1(initialCurve, 100);
    println(transformedCurve.size() + " points.");
  }
  else if(key == 'e') {
    print("Curve regularization 2: ");
    transformedCurve = curveRegularization2(initialCurve, 100, 3);
    println(transformedCurve.size() + " points.");
  }
  else if(key == 'r') {
    print("Curve regularization 3: ");
    transformedCurve = curveRegularization3(initialCurve, 100, 3, 10);
    println(transformedCurve.size() + " points.");
  }
  else if(key == 't') {
    print("Curve regularization 3 + Douglas Peucker: ");
    transformedCurve = DouglasPeucker(curveRegularization3(initialCurve, 100, 3, 10), 10);
    println(transformedCurve.size() + " points.");
  }
  else if(key == 'c') {
    initialCurve.clear();
    transformedCurve.clear();
  }
}

void drawCurve(ArrayList<PVector> points, color curveColor, boolean drawPoints, int step) {
  if(points.size() < 2)
    return;
      
  strokeWeight(2);
  stroke(curveColor);
  PVector previous = points.get(0);
  for(int i = step; i < points.size(); i+=step) {
    PVector current = points.get(i);
    line(previous.x, -previous.y, current.x, -current.y);
    previous = current;
  }
  
  if(drawPoints) {
    noStroke();
    fill(0);
    for(int i = 0; i < points.size(); i+=step) {
      PVector point = points.get(i);
      ellipse(point.x, -point.y, 5, 5);
    }
  }
}

ArrayList<PVector> randomCurve(int pointsCount, PVector center, float minRadius, float maxRadius) {
  final float amplitude = maxRadius - minRadius;
  final float smallNoiseAmplitude = 0.1f * amplitude;
  final float bigNoiseAmplitude = 0.5f * amplitude;
  
  ArrayList<PVector> points = new ArrayList<PVector>(pointsCount);
  float radius = random(minRadius, maxRadius);
  
  PVector d = new PVector(1, 0);
  for(int i = 0; i < pointsCount; i++) {
    if(random(pointsCount) < 20f)
      radius += random(-bigNoiseAmplitude, bigNoiseAmplitude);
    radius += random(-smallNoiseAmplitude, smallNoiseAmplitude);
    
    radius = constrain(radius, minRadius, maxRadius);
    PVector point = PVector.mult(d, radius, new PVector()).add(center);
    point.y = -point.y;
    
    points.add(point);
    d.rotate(-TWO_PI / pointsCount);
  }
  
  return points;
}

void benchmarks() {
  final int pointsCount = 50000;
  
  ArrayList<PVector> curve = randomCurve(pointsCount, new PVector(width/2, height/2), height/10, height/2);
  
  long t = System.currentTimeMillis();
  ArrayList<PVector> result = curveRegularization1(curve, 10);
  t = System.currentTimeMillis() - t;
  
  println("Curve regularization 1: " + pointsCount + " => " + result.size() + " (" + t + " millis).");
  
  t = System.currentTimeMillis();
  result = curveRegularization2(curve, 10, 5);
  t = System.currentTimeMillis() - t;
  
  println("Curve regularization 2: " + pointsCount + " => " + result.size() + " (" + t + " millis).");
  
  t = System.currentTimeMillis();
  result = curveRegularization3(curve, 10, 5, 10);
  t = System.currentTimeMillis() - t;
  
  println("Curve regularization 3: " + pointsCount + " => " + result.size() + " (" + t + " millis).");
  
  t = System.currentTimeMillis();
  result = DouglasPeucker(curveRegularization3(curve, 10, 5, 10), 20);
  t = System.currentTimeMillis() - t;
  
  println("Curve regularization 3 + Douglas Peucker: " + pointsCount + " => " + result.size() + " (" + t + " millis).");
}
