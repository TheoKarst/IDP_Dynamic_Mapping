PVector lidarCenter;
Observation[] observations;

boolean filter = false;
float filterAngle = 10;    // Angle in degrees

// Register key pressed:
boolean keyUpPressed = false;
boolean keyDownPressed = false;

void setup() {
  size(2000, 1200, P2D);  
  lidarCenter = new PVector(1800, 100);
  observations = LoadObservations("lidar_data.csv", 1, 30, HALF_PI);
  
  // benchmarks();
}

void draw() {
  background(255);
  
  if(keyDownPressed && filterAngle > 0) filterAngle--;
  else if(keyUpPressed && filterAngle < 180) filterAngle++;
  
  
  // drawConeLine(width/10, height/2, width/10 + 400, height/2, radians(filterAngle));
  // drawConeLine(width/10, height/2, mouseX, height - mouseY, radians(filterAngle));
  
  stroke(0);
  DrawObservations(observations);
  
  if(filter) {
    println("Filter angle: " + filterAngle + "°");
    
    Observation nearest = null;
    float minDistSq = -1;
    for(Observation observation : observations) {
      float dX = observation.x - mouseX, dY = observation.y - (height-mouseY);
      float distSq = dX*dX + dY*dY;
      
      if(nearest == null || distSq < minDistSq) {
        nearest = observation;
        minDistSq = distSq;
      }
    }
    
    DrawAlphaCrop(nearest, radians(filterAngle));
    
    // int[] indices = AlphaFilter_Naive(observations, radians(filterAngle));
    int[] indices = AlphaFilter_Improved(observations, radians(filterAngle));
    
    noStroke();
    fill(255, 0, 0);
    for(int i = 0; i < indices.length; i++) {
      Observation observation = observations[indices[i]];
      drawPoint(observation.x, observation.y, 5);
    }
  }
}

void keyPressed() {
  if(key == 'f') filter = !filter;
    
  if(keyCode == UP) keyUpPressed = true;
  if(keyCode == DOWN) keyDownPressed = true;
}

void keyReleased() {
  keyUpPressed = false;
  keyDownPressed = false;
}

Observation[] LoadObservations(String filename, int firstLine, float scaleFactor, float offsetAngle) {
  String[] lines = loadStrings(filename);
  Observation[] observations = new Observation[lines.length - firstLine];
  
  for(int i = firstLine; i < lines.length; i++) {
    String[] data = lines[i].replace(",", ".").split(";");
    observations[i - firstLine] = new Observation(scaleFactor * float(data[0]), offsetAngle + float(data[1]));
  }
  
  return observations;
}

void DrawObservations(Observation[] observations) {
  for(Observation observation : observations)    
    drawLine(lidarCenter.x, lidarCenter.y, observation.x, observation.y);
}

void DrawAlphaCrop(Observation[] observations, float alpha) {
  final int linesLength = 2 * width;
  
  stroke(255, 0, 0);
  for(Observation observation : observations) {
    float theta = observation.theta;
    float x = observation.x, y = observation.y;
    
    PVector u = new PVector(cos(theta + alpha), sin(theta + alpha));
    PVector v = new PVector(cos(theta - alpha), sin(theta - alpha));
    
    drawLine(x, y, x + linesLength * u.x, y + linesLength * u.y);
    drawLine(x, y, x + linesLength * v.x, y + linesLength * v.y);
  }
}

void DrawAlphaCrop(Observation observation, float alpha) {
  final int linesLength = 2 * width;
  
  stroke(255, 0, 0);
  float theta = observation.theta;
  float x = observation.x, y = observation.y;
  
  PVector u = new PVector(cos(theta + alpha), sin(theta + alpha));
  PVector v = new PVector(cos(theta - alpha), sin(theta - alpha));
  
  drawLine(x, y, x + linesLength * u.x, y + linesLength * u.y);
  drawLine(x, y, x + linesLength * v.x, y + linesLength * v.y);
}

public void drawLine(float x0, float y0, float x1, float y1) {
  line(x0, height - y0, x1, height - y1);
}

public void drawPoint(float x, float y, float radius) {
  ellipse(x, height - y, 2*radius, 2*radius);
}

public void drawConeLine(float x0, float y0, float x1, float y1, float alpha) {
  stroke(0);
  drawLine(x0, y0, x1, y1);
  
  float theta = atan2(y1 - y0, x1 - x0);
  PVector u = new PVector(cos(theta + alpha), sin(theta + alpha));
  PVector v = new PVector(cos(theta - alpha), sin(theta - alpha));
  
  
  final int linesLength = 2 * width;
  
  stroke(255, 0, 0);
  drawLine(x1, y1, x1 + linesLength * u.x, y1 + linesLength * u.y);
  drawLine(x1, y1, x1 + linesLength * v.x, y1 + linesLength * v.y);
}

Observation[] InflateObservations(Observation[] observations, int count) {
  Observation[] result = new Observation[(observations.length-1) * count + 1];
  
  for(int i = 0; i < observations.length - 1; i++) {
    Observation current = observations[i];
    Observation next = observations[i+1];
    
    for(int j = 0; j < count; j++) {
      float r = map(j, 0, count, current.r, next.r);
      float theta = map(j, 0, count, current.theta, next.theta);
      result[count * i + j] = new Observation(r, theta);
    }
  }
  
  result[result.length-1] = observations[observations.length-1];
  
  return result;
}

void benchmarks() {
  Observation[] test = InflateObservations(observations, 50);
  
  println("Running the algorithm on a set of " + test.length + " points...");
  
  int[] angles = new int[]{1, 5, 10, 20, 40, 60, 80};
  
  for(int angle : angles) {    
    long t1 = System.currentTimeMillis();
    int[] indices1 = AlphaFilter_Naive(test, radians(angle));
    t1 = System.currentTimeMillis() - t1;
    
    long t2 = System.currentTimeMillis();
    int[] indices2 = AlphaFilter_Improved(test, radians(angle));
    t2 = System.currentTimeMillis() - t2;
    
    boolean sameResult = indices1.length == indices2.length;
    if(sameResult) {
      for(int i = 0; i < indices1.length && sameResult; i++)
        if(indices1[i] != indices2[i])
          sameResult = false;
    }
    
    print("Using alpha = " + angle + "°: Naïve: " + t1 + " millis, Improved: " + t2 + " millis. ");
    if(sameResult)
      println("Same result: " + indices1.length + " points !");
    else
      println("Different results: Naïve: " + indices1.length + " points, Improved: " + indices2.length + " points.");
  }
}
