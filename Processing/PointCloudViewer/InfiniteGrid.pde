public class InfiniteGrid {
  private final int SMALL_GRID_SIZE = 50;
  private final int LARGE_GRID_SIZE = 40;
  
  // The colors for Blender are:
  // Background: color(53)
  // Small grid: color(60)
  // Large grid: color(76)
  private final int SMALL_GRID_COLOR = color(70);
  private final int LARGE_GRID_COLOR = color(86);
  
  private int GRID_SCALE = 10;
  
  // Position and orientation of the grid:
  private PMatrix3D transform = new PMatrix3D();
  
  public InfiniteGrid(){
    // For testing only:
    //transform.translate(width/2, height/2, -100);
    //transform.rotateX(4*PI/10);
    //transform.rotateZ(PI/5);
  }
  
  public void drawGrid(PMatrix3D cameraTransform){
    // Compute the orthogonal projection of the camera on the grid, and draw the grid only around the camera:
    PVector cameraProj = globalToLocalCoordinates(cameraTransform.m03, cameraTransform.m13, cameraTransform.m23);
    drawGrid((int) cameraProj.x, (int) cameraProj.y);
  }
  
  private void drawGrid(int centerX, int centerY){
    strokeWeight(1);
    
    // u, v and w are the vectors of the grid, in world space coordinates:
    PVector u = new PVector(transform.m00, transform.m10, transform.m20);
    PVector v = new PVector(transform.m01, transform.m11, transform.m21);
    PVector w = new PVector(transform.m02, transform.m12, transform.m22);
    
    // Align (centerX, centerY) with the grid:
    centerX = centerX - centerX % (10 * GRID_SCALE);
    centerY = centerY - centerY % (10 * GRID_SCALE);
    
    // The grid world space position is (transform.m03, transform.m13, transform.m23).
    // But to have a pseudo infinite grid, we only draw the part of the grid around the giving center (in grid coordinates):
    
    // Draw the small grid:
    // stroke(SMALL_GRID_COLOR);
    // drawGrid(u, v, centerX, centerY, SMALL_GRID_SIZE, GRID_SCALE, true, SMALL_GRID_COLOR);
    
    // Draw the large grid:
    stroke(LARGE_GRID_COLOR);
    drawGrid(u, v, centerX, centerY, LARGE_GRID_SIZE, 10 * GRID_SCALE, false, LARGE_GRID_COLOR);
    
    // Draw the Z axis of the grid in blue:
    stroke(0, 0, 255);
    int totalSize = 10 * LARGE_GRID_SIZE * GRID_SCALE;
    line(transform.m03 + w.x*totalSize, transform.m13 + w.y*totalSize, transform.m23 + w.z*totalSize, 
         transform.m03 - w.x*totalSize, transform.m13 - w.y*totalSize, transform.m23 - w.z*totalSize);
  }
  
  private void drawGrid(PVector u, PVector v, int centerX, int centerY, int gridSize, int gridScale, boolean smallGrid, int gridColor){
    int totalSize = gridSize * gridScale;
    int gridMinX = centerX-totalSize;
    int gridMaxX = centerX+totalSize;
    int gridMinY = centerY-totalSize;
    int gridMaxY = centerY+totalSize;
    
    
    for(int i = -gridSize; i <= gridSize; i++){
      if(smallGrid && i % 10 == 0)
        continue;
      
      int hStepI = centerY + i * gridScale;
      int vStepI = centerX + i * gridScale;
      
      float hStartX = transform.m03 + gridMinX*u.x + hStepI*v.x;
      float hStartY = transform.m13 + gridMinX*u.y + hStepI*v.y;
      float hStartZ = transform.m23 + gridMinX*u.z + hStepI*v.z;
      
      float hEndX = transform.m03 + gridMaxX*u.x + hStepI*v.x;
      float hEndY = transform.m13 + gridMaxX*u.y + hStepI*v.y;
      float hEndZ = transform.m23 + gridMaxX*u.z + hStepI*v.z;
      
      float vStartX = transform.m03 + vStepI * u.x + gridMinY * v.x;
      float vStartY = transform.m13 + vStepI * u.y + gridMinY * v.y;
      float vStartZ = transform.m23 + vStepI * u.z + gridMinY * v.z;
      
      float vEndX = transform.m03 + vStepI * u.x + gridMaxY * v.x;
      float vEndY = transform.m13 + vStepI * u.y + gridMaxY * v.y;
      float vEndZ = transform.m23 + vStepI * u.z + gridMaxY * v.z;
      
      // If we are currently drawing the X axis, draw it in red:
      stroke(hStepI == 0 ? color(255, 0, 0) : gridColor);
      line(hStartX, hStartY, hStartZ, hEndX, hEndY, hEndZ);
      
      // If we are currently drawing the Y axis, draw it in green:
      stroke(vStepI == 0 ? color(0, 255, 0) : gridColor);
      line(vStartX, vStartY, vStartZ, vEndX, vEndY, vEndZ);      
    }
  }
  
  // Return the intersection of the given ray and this grid in world space coordinates, or null if no intersection:
  public PVector rayIntersect(PVector rayStart, PVector direction){
    // Let O = center of the grid, P = rayStart and n = vector normal to the plane.
    // We search the intersection I between the ray and the plane: let t such that PI = t*direction.
    // I belongs to the plane, so (OP + t*direction).dot(n) = 0 <=> t = -OP.dot(n) / direction.dot(n)
    
    // Vector normal to the plane:
    PVector n = new PVector(transform.m02, transform.m12, transform.m22);
    
    // Compute the dot product between the direction of the ray, and the vector normal to the plane:
    float p1 = direction.dot(n);
    
    // If the direction is parallel to the plane, there is no intersection:
    if(p1 == 0)
      return null;
      
    // Vector from the center of the grid to the start position of the ray:
    PVector OP = new PVector(
      rayStart.x - transform.m03,
      rayStart.y - transform.m13,
      rayStart.z - transform.m23);
    
    // Compute t such that PI = t*direction:
    float t = -OP.dot(n)/p1;
    
    // If t <= 0, the ray goes in the wrong direction, so there is no intersection:
    if(t <= 0)
      return null;
      
    return rayStart.copy().add(direction.mult(t));
  }
  
  // Transform the position of a point from global to local space:
  public PVector globalToLocalCoordinates(PVector point){
    return globalToLocalCoordinates(point.x, point.y, point.z);
  }
  
  public PVector globalToLocalCoordinates(float xP, float yP, float zP){
    // Vector from the center of the grid to the point:
    PVector v = new PVector(
      xP - transform.m03,
      yP - transform.m13,
      zP - transform.m23);
    
    PVector x = new PVector(transform.m00, transform.m10, transform.m20);
    PVector y = new PVector(transform.m01, transform.m11, transform.m21);
    PVector z = new PVector(transform.m02, transform.m12, transform.m22);
    
    return new PVector(v.dot(x), v.dot(y), v.dot(z));
  }
  
  // Transform the position of a point from local to global space:
  public PVector localToGlobalCoordinates(PVector point){
    return localToGlobalCoordinates(point.x, point.y, point.z);
  }
  
  public PVector localToGlobalCoordinates(float xP, float yP, float zP){
    PVector center = new PVector(transform.m03, transform.m13, transform.m23);
    PVector x = new PVector(transform.m00, transform.m10, transform.m20).mult(xP);
    PVector y = new PVector(transform.m01, transform.m11, transform.m21).mult(yP);
    PVector z = new PVector(transform.m02, transform.m12, transform.m22).mult(zP);
    
    return center.add(x).add(y).add(z);
  }
  
  public void setTransform(PMatrix3D transform){
    this.transform = transform;
  }
}
