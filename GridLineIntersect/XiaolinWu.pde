int ipart(float x) {
  return (int) x;
}

float fpart(float x) {
  return x - floor(x);
}

float rfpart(float x) {
  return 1.0 - fpart(x);
}

void setPixel(float x, float y, float brightness) {
  fill(255 * (1-brightness));
  // if(brightness >= 0.5f)
  rect(int(x+0.5) * pixelSize, int(y+0.5) * pixelSize, pixelSize, pixelSize);
}

void drawLineXiaolinWu(float x0, float y0, float x1, float y1) {
  boolean steep = abs(y1 - y0) > abs(x1 - x0);

  if (steep) {
    // Swap(x0, y0):
    float tmp = x0;
    x0 = y0;
    y0 = tmp;

    // Swap(x1, y1):
    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }
  if (x0 > x1) {
    // Swap(x0, x1)
    float tmp = x0;
    x0 = x1;
    x1 = tmp;

    // Swap (y0, y1):
    tmp = y0;
    y0 = y1;
    y1 = tmp;
  }

  float dx = x1 - x0;
  float dy = y1 - y0;

  float gradient = dx == 0 ? 1 : dy / dx;

  // Handle first endpoint
  float xend = round(x0);
  float yend = y0 + gradient * (xend - x0);
  float xgap = rfpart(x0 + 0.5);
  float xpxl1 = xend;  // this will be used in the main loop
  float ypxl1 = ipart(yend);
  if (steep) {
    setPixel(ypxl1, xpxl1, rfpart(yend) * xgap);
    setPixel(ypxl1+1, xpxl1, fpart(yend) * xgap);
  } else {
    setPixel(xpxl1, ypxl1, rfpart(yend) * xgap);
    setPixel(xpxl1, ypxl1+1, fpart(yend) * xgap);
  }
  float intery = yend + gradient; // first y-intersection for the main loop

  // handle second endpoint
  xend = round(x1);
  yend = y1 + gradient * (xend - x1);
  xgap = fpart(x1 + 0.5);
  float xpxl2 = xend; //this will be used in the main loop
  float ypxl2 = ipart(yend);
  if (steep) {
    setPixel(ypxl2, xpxl2, rfpart(yend) * xgap);
    setPixel(ypxl2+1, xpxl2, fpart(yend) * xgap);
  } else {
    setPixel(xpxl2, ypxl2, rfpart(yend) * xgap);
    setPixel(xpxl2, ypxl2+1, fpart(yend) * xgap);
  }

  // main loop
  if (steep) {
    for (float x = xpxl1 + 1; x <= xpxl2 - 1; x++) {
      setPixel(ipart(intery), x, rfpart(intery));
      setPixel(ipart(intery)+1, x, fpart(intery));
      intery = intery + gradient;
    }
  } else {
    for (float x = xpxl1 + 1; x <= xpxl2 - 1; x++) {
      setPixel(x, ipart(intery), rfpart(intery));
      setPixel(x, ipart(intery)+1, fpart(intery));
      intery = intery + gradient;
    }
  }
}


// Same as Xiaolin Wu's algorithm, but ignore the pixels brightness and change
// the coordinate system of the pixels to be corner left aligned:
void drawStrokeLine(float x0, float y0, float x1, float y1) {
  boolean steep = abs(y1 - y0) > abs(x1 - x0);

  if (steep) {
    float tmp = x0; x0 = y0; y0 = tmp;  // Swap(x0, y0)
    tmp = x1; x1 = y1; y1 = tmp;        // Swap(x1, y1)
  }
  if (x0 > x1) {
    float tmp = x0; x0 = x1; x1 = tmp;  // Swap(x0, x1)
    tmp = y0; y0 = y1; y1 = tmp;        // Swap(y0, y1)
  }

  float dx = x1 - x0, dy = y1 - y0;
  float gradient = dx == 0 ? 1 : dy / dx;

  // Handle first endpoint:
  int xend = floor(x0);
  float yend = y0 + gradient * (xend - x0) + (gradient - 1) / 2;
  int xpxl1 = xend;  // This will be used in the main loop
  int ypxl1 = floor(yend);
  if (steep) {
    setPixel(ypxl1, xpxl1);
    setPixel(ypxl1+1, xpxl1);
  } else {
    setPixel(xpxl1, ypxl1);
    setPixel(xpxl1, ypxl1+1);
  }
  
  // First y-intersection for the main loop:
  float intery = yend + gradient;

  // Handle second endpoint:
  xend = floor(x1);
  yend = y1 + gradient * (xend - x1) + (gradient - 1) / 2;
  int xpxl2 = xend;  // This will be used in the main loop
  int ypxl2 = floor(yend);
  if (steep) {
    setPixel(ypxl2, xpxl2);
    setPixel(ypxl2+1, xpxl2);
  } else {
    setPixel(xpxl2, ypxl2);
    setPixel(xpxl2, ypxl2+1);
  }

  // main loop
  if (steep) {
    for (int x = xpxl1 + 1; x < xpxl2; x++) {
      int y = floor(intery);

      setPixel(y, x);
      setPixel(y+1, x);
      intery = intery + gradient;
    }
  }
  else {
    for (int x = xpxl1 + 1; x < xpxl2; x++) {
      int y = floor(intery);

      setPixel(x, y);
      setPixel(x, y+1);
      intery = intery + gradient;
    }
  }
}
