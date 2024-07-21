// Implementation of the Digital Differential Analyzer algorithm:

void drawLineDDA(float x0, float y0, float x1, float y1) {  
  float dx = x1 - x0;
  float dy = y1 - y0;
  
  int steps = ceil(abs(dx) == abs(dy) ? abs(dx) : abs(dx) + abs(dy));

  float xIncrement = dx / steps;
  float yIncrement = dy / steps;

  float x = x0;
  float y = y0;

  for (int i = 0; i <= steps; i++) {
    int ix = floor(x);
    int iy = floor(y);
    setPixel(ix, iy);

    x += xIncrement;
    y += yIncrement;
  }
}
