int pixelSize = 20;

float centerX, centerY;

int methodIndex = 0;
String[] methods = new String[]{
  "Bresenham",                        // 0
  "Voxel Traversal",                  // 1
  "Digital Differential Analyzer",    // 2
  "Xiaolin Wu - Brightness",          // 3
  "Xiaolin Wu - no brightness",       // 4
  
};

void setup() {
  size(1600, 1000, P2D);
  noStroke();
  fill(0);
  
  centerX = width / (2 * pixelSize) + 0.5f;
  centerY = height / (2 * pixelSize) + 0.5f;
  
  println("Current method: " + methods[methodIndex]);
}

void draw() {
  background(255);
  
  noStroke();
  fill(0);
  
  if(methodIndex == 0)
    drawLineBresenham((int) centerX, (int) centerY, mouseX / pixelSize, mouseY / pixelSize);
    
  else if(methodIndex == 1)
    drawLineVoxelTraversal(centerX, centerY, float(mouseX) / pixelSize, float(mouseY) / pixelSize);
    
  else if(methodIndex == 2)
    drawLineDDA(centerX, centerY, float(mouseX) / pixelSize, float(mouseY) / pixelSize);
    
  else if(methodIndex == 3)
    drawLineXiaolinWu(centerX - 0.5f, centerY - 0.5f, float(mouseX) / pixelSize - 0.5, float(mouseY) / pixelSize - 0.5);
    
  else if(methodIndex == 4)
    drawStrokeLine(centerX, centerY, float(mouseX) / pixelSize, float(mouseY) / pixelSize);
  
  stroke(255, 0, 0);
  strokeWeight(2);
  line(pixelSize * centerX, pixelSize * centerY, mouseX, mouseY);
}

void keyPressed() {
  if(keyCode == UP) centerY -= 0.1f;
  if(keyCode == DOWN) centerY += 0.1f;
  if(keyCode == LEFT) centerX -= 0.1f;
  if(keyCode == RIGHT) centerX += 0.1f;
  
  if(key == ' ') {
    methodIndex = (methodIndex + 1) % methods.length;
    println("Current method: " + methods[methodIndex]);
  }
}

void setPixel(int x, int y) {
  rect(x * pixelSize, y * pixelSize, pixelSize, pixelSize);
}
