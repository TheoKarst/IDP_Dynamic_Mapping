int pixelSize = 4;

PVector v = new PVector(40, 0);

void setup() {
  size(800, 800);
  noStroke();
  fill(0);
}

void draw() {
  background(255);
  
  v.rotate(PI/200);
  
  int centerX = width / (2 * pixelSize);
  int centerY = height / (2 * pixelSize);
  
  drawLine(centerX, centerY, centerX + (int) v.x, centerY + (int) v.y);
}

void drawLine(int x0, int y0, int x1, int y1) {
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
  int err = (dx>dy ? dx : -dy)/2, e2;

  while(true) {
    setPixel(x0,y0);
    
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}

void setPixel(int x, int y) {
  rect(x * pixelSize, y * pixelSize, pixelSize, pixelSize);
}
