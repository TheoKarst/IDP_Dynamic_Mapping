ArrayList<PVector> points = new ArrayList<PVector>();
float Rx = 0, Ry = 0, Rxx = 0, Ryy = 0, Rxy = 0;
float rho = 0, theta = 0;

// TEST:
double dRx = 0, dRy = 0, dRxx = 0, dRyy = 0, dRxy = 0;


void setup() {
  size(2000, 1000, P2D);
  ellipseMode(CENTER);
}

void draw() {
  background(255);
  
  final float lineExtent = 5000;
  float costheta = cos(theta), sintheta = sin(theta);
  float centerX = rho * costheta;
  float centerY = rho * sintheta;
  
  stroke(255, 0, 0);
  line(centerX - lineExtent * sintheta, height - (centerY + lineExtent * costheta),
       centerX + lineExtent * sintheta, height - (centerY - lineExtent * costheta));
  
  noStroke();
  fill(0, 0, 255);
  for(PVector point : points)
    ellipse(point.x, height-point.y, 10, 10);
}

void keyPressed() {
  if(key == 'c')
    Clear();
}

void mousePressed() {
  AddPoint(new PVector(mouseX, height-mouseY));
}

void AddPoint(PVector point) {
    println("(" + point.x + "; " + point.y + ")");
    points.add(point);

    Rx += point.x;
    Ry += point.y;
    Rxx += point.x * point.x;
    Ryy += point.y * point.y;
    Rxy += point.x * point.y;
    
    dRx += point.x;
    dRy += point.y;
    dRxx += point.x * point.x;
    dRyy += point.y * point.y;
    dRxy += point.x * point.y;
    
    int n = points.size();
    float N1 = Rxx * n - Rx * Rx;
    float N2 = Ryy * n - Ry * Ry;
    float T = Rxy * n - Rx * Ry;
    
    double dN1 = dRxx * n - dRx * dRx;
    double dN2 = dRyy * n - dRy * dRy;
    double dT = dRxy * n - dRx * dRy;
    

    // N1 and N2 represent the width of the cloud of the regression points along the X- and Y-axis. If N1 is larger 
    // than N2, the cloud of points lies more horizontally than vertically which makes regression of y to x (y = mx + q)
    // more favourable. Otherwise, the regression of x to y is selected (x = sy + t):
    if (N1 >= N2) {
        float m = T / N1;
        float q = (Ry - m * Rx) / n;

        rho = abs(q / sqrt(m * m + 1));
        theta = atan2(q, -q * m);
    }
    else {
        float s = T / N2;
        float t = (Rx - s * Ry) / n;

        rho = abs(t / sqrt(s * s + 1));
        theta = atan2(-t * s, t);
    }
    
    // Compute R-squared:
    float R = T * T / (N1 * N2);
    double dR = dT * dT / (dN1 * dN2);
    
    println("R: " + R + " (double: " + dR + ")");
}

void Clear() {
  points.clear();
  Rx = Ry = Rxx = Ryy = Rxy = 0;
  dRx = dRy = dRxx = dRyy = dRxy = 0;
  rho = theta = 0;
}
