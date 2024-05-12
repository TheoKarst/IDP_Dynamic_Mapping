ArrayList<PVector> points = new ArrayList<PVector>();
ArrayList<PVector> hull = null;

void setup() {
  size(1000, 600);
  ellipseMode(CENTER);
}

void draw() {
  background(255);
  
  stroke(0);
  fill(0, 0, 255);
  for(PVector point : points)
    ellipse(point.x, point.y, 10, 10);
    
  if(hull != null) {
    stroke(0, 255, 0);
    PVector prev = hull.get(hull.size()-1);
    
    for(PVector point : hull) {
      line(prev.x, prev.y, point.x, point.y);
      prev = point;
    }
  }
}

void mousePressed() {
  points.add(new PVector(mouseX, mouseY));
}

void keyPressed() {
  if(key == ENTER)
    hull = giftWrapping(points);
    
  else if(key == 'c') {
    points.clear();
    hull = null;
  }
}

ArrayList<PVector> giftWrapping(ArrayList<PVector> points) {
  ArrayList<PVector> hull = new ArrayList<PVector>();
  
  // Find the leftmost point:
  PVector leftmost = points.get(0);
  for(int i = 1; i < points.size(); i++) {
    PVector current = points.get(i);
    if(current.x < leftmost.x)
      leftmost = current;
  }
  
  PVector tmp = new PVector();
  PVector pointOnHull = leftmost;
  do {
    hull.add(pointOnHull);
    println(pointOnHull);
    
    PVector endpoint = points.get(0);
    PVector n = new PVector(pointOnHull.y - endpoint.y, endpoint.x - pointOnHull.x);
    
    for(PVector point : points) {
      if(endpoint == pointOnHull || PVector.sub(point, pointOnHull, tmp).dot(n) > 0) {
        endpoint = point;
        n.set(pointOnHull.y - endpoint.y, endpoint.x - pointOnHull.x);
      }
    }
    pointOnHull = endpoint;
  }
  while(pointOnHull != leftmost);
  
  return hull;
}
