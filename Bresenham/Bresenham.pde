int pixelSize = 10;

PVector v = new PVector(30, 0);

boolean R_pressed = false;

void setup() {
  size(1600, 1000, P2D);
  noStroke();
  fill(0);
}

void draw() {
  background(255);
  
  float centerX = width / (2 * pixelSize) + 0.25f;
  float centerY = height / (2 * pixelSize) + 0.25f;
  
  noStroke();
  fill(0);
  drawLineBresenham2(centerX, centerY, centerX + v.x, centerY + v.y);
  
  stroke(255, 0, 0);
  strokeWeight(2);
  line(pixelSize * centerX, pixelSize * centerY, pixelSize * (centerX + v.x), pixelSize * (centerY + v.y));
  
  if(R_pressed)
    v.rotate(PI / 300);
}

void keyPressed() {
  if(key == 'r') R_pressed = true;
}

void keyReleased() {
  if(key == 'r') R_pressed = false;
}

void drawLineBresenham(int x0, int y0, int x1, int y1) {
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

void drawLineBresenham2(float x0, float y0, float x1, float y1) {  
  int x = (int) x0, y = (int) y0;
  int xEnd = (int) x1, yEnd = (int) y1;
  
  int stepX = x0 <= x1 ? 1 : -1;
  int stepY = y0 <= y1 ? 1 : -1;
  
  if(x == xEnd) {
    setPixel(x, y);
    while(y != yEnd) {
      y += stepY;
      setPixel(x, y);
    }
    return;
  }
  
  if(y == yEnd) {
    setPixel(x, y);
    while(x != xEnd) {
      x += stepX;
      setPixel(x, y);
    }
    return;
  }
  
  float tMaxX = (x0 < x1 ? x + stepX - x0 : x - x0) / (x1 - x0);
  float tMaxY = (y0 < y1 ? y + stepY - y0 : y - y0) / (y1 - y0);
  
  float slopeX = 1/abs(x1-x0);
  float slopeY = 1/abs(y1-y0);
  
  
  while(true) {
    setPixel(x, y);
    
    if(x==xEnd && y==yEnd) break;
    
    if(tMaxX < tMaxY) {
      tMaxX += slopeX;
      x += stepX;
    }
    else {
      tMaxY += slopeY;
      y += stepY;
    }
  }
}

void drawLineDDA(float x0, float y0, float x1, float y1) {  
  float dx = x1 - x0;
  float dy = y1 - y0;
  println(dx + ";" + dy);
  
  float steps = abs(dx) == abs(dy) ? abs(dx) : abs(dx) + abs(dy);

  float xIncrement = dx / steps;
  float yIncrement = dy / steps;

  float x = x0;
  float y = y0;

  for (int i = 0; i <= steps; i++) {
    int ix = round(x);
    int iy = round(y);
    setPixel(ix, iy);

    x += xIncrement;
    y += yIncrement;
  }
}

void setPixel(int x, int y) {
  rect(x * pixelSize, y * pixelSize, pixelSize, pixelSize);
}
