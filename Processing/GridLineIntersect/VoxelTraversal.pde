// Implementation of the algorithm described by the following paper:
// https://www.cse.chalmers.se/edu/year/2013/course/TDA361/grid.pdf

void drawLineVoxelTraversal(float x0, float y0, float x1, float y1) {  
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
  
  int stepsCount = abs(xEnd-x) + abs(yEnd-y);
  for(int i = 0; i <= stepsCount; i++) {
    setPixel(x, y);
    
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
