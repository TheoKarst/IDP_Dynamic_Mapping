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
  //if(brightness >= 0.5f)
    rect(int(x) * pixelSize, int(y) * pixelSize, pixelSize, pixelSize);
}

void drawLineXiaolinWu(float x0, float y0, float x1, float y1) {

  boolean steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
    drawLineXiaolinWu(y0, x0, y1, x1);

  if (x0 > x1)
    drawLineXiaolinWu(x1, y1, x0, y0);

  float dx = x1 - x0;
  float dy = y1 - y0;
  float gradient = dy / dx;

  // handle first endpoint
  float xend = round(x0);
  float yend = y0 + gradient * (xend - x0);
  float xgap = rfpart(x0 + 0.5);
  float xpxl1 = xend; // this will be used in the main loop
  float ypxl1 = ipart(yend);

  if (steep) {
    setPixel(ypxl1, xpxl1, rfpart(yend) * xgap);
    setPixel(ypxl1 + 1, xpxl1, fpart(yend) * xgap);
  } else {
    setPixel(xpxl1, ypxl1, rfpart(yend) * xgap);
    setPixel(xpxl1, ypxl1 + 1, fpart(yend) * xgap);
  }

  // first y-intersection for the main loop
  float intery = yend + gradient;

  // handle second endpoint
  xend = round(x1);
  yend = y1 + gradient * (xend - x1);
  xgap = fpart(x1 + 0.5);
  float xpxl2 = xend; // this will be used in the main loop
  float ypxl2 = ipart(yend);

  if (steep) {
    setPixel(ypxl2, xpxl2, rfpart(yend) * xgap);
    setPixel(ypxl2 + 1, xpxl2, fpart(yend) * xgap);
  } else {
    setPixel(xpxl2, ypxl2, rfpart(yend) * xgap);
    setPixel(xpxl2, ypxl2 + 1, fpart(yend) * xgap);
  }

  // main loop
  for (float x = xpxl1 + 1; x <= xpxl2 - 1; x++) {
    if (steep) {
      setPixel(ipart(intery), x, rfpart(intery));
      setPixel(ipart(intery) + 1, x, fpart(intery));
    } else {
      setPixel(x, ipart(intery), rfpart(intery));
      setPixel(x, ipart(intery) + 1, fpart(intery));
    }
    intery = intery + gradient;
  }
}
