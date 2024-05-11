final int POINTS_COUNT = 1000;
PVector points[] = new PVector[POINTS_COUNT];

void setup() {
  size(2000, 1200, P2D);
  textSize(32);
  
  float dX = float(width) / points.length;
  for(int i = 0; i < points.length; i++) {
    float x = dX * i;
    float y = map(f(0.005f * i), -1, 1, height, 0);
    points[i] = new PVector(x, y);
  }
}

void draw() {
  background(255);
  
  float epsilon = mouseX * mouseX / 10000f;
  
  strokeWeight(2);
  stroke(0, 0, 255);
  drawCurve(points);
  
  PVector[] simplify = DouglasPeucker(points, 0, points.length-1, epsilon);
  
  stroke(0);
  strokeWeight(4);
  drawCurve(simplify);
  
  fill(0);
  text("Epsilon: " + epsilon, 10, 40);
  text("Initial curve: " + points.length + " points", 10, 80);
  text("Simplified curve: " + simplify.length + " points", 10, 120);
}

void drawCurve(PVector points[]) {
  PVector prev = points[0];
  for(int i = 1; i < points.length; i++) {
    PVector curr = points[i];
    line(prev.x, prev.y, curr.x, curr.y);
    prev = curr;
  }
}

float f(float x) {
  return cos(TWO_PI*x)*exp(-x);
}

// Run douglas peucker algorithm on the given list of points, between start and end (inclusives):
PVector[] DouglasPeucker(PVector[] points, int start, int end, float epsilon) {
  PVector u = PVector.sub(points[start], points[end], new PVector());
  u.normalize();
  
  // Find the point with maximum orthogonal distance:
  int index = -1;
  float maxDistance = 0;
  
  // Tempoary variables to compute the orthogonal distance:
  PVector v = new PVector();
  PVector v_along_u = new PVector();
  
  for(int i = start+1; i < end; i++) {
    PVector.sub(points[i], points[start], v);
    PVector.mult(u, v.dot(u), v_along_u);
    float distance = PVector.sub(v, v_along_u).mag();
    
    if(distance >= maxDistance) {
      maxDistance = distance;
      index = i;
    }
  }
  
  if(maxDistance > epsilon) {
    PVector list1[] = DouglasPeucker(points, start, index, epsilon);
    PVector list2[] = DouglasPeucker(points, index, end, epsilon);
    
    PVector result[] = new PVector[list1.length + list2.length - 1];
    for(int i = 0; i < list1.length-1; i++) result[i] = list1[i];
    for(int i = 0; i < list2.length; i++) result[list1.length+i-1] = list2[i];
    
    return result;
  }
  else
    return new PVector[]{points[start], points[end]};
}
