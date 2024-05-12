final float MAX_DISTANCE = 20;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Static clusters are used to represent a set of points that are near to each other.
// These clusters are well suited to represent the current clusters on a frame (without taking into account the previous frames),
// and new static clusters are therefore created each frame to match the current set of points:

class StaticCluster {
  private final float DOUGLAS_PEUCKER_EPSILON = 1;
  
  // List of dynamic clusters that are inside this static cluster:
  private ArrayList<DynamicCluster> matches = new ArrayList<DynamicCluster>();
  
  private final ArrayList<PVector> points;
  
  private PVector u, v;                        // Axis of the bounding rectangle
  private float centerX, centerY;              // Center of the bounding rectangle
  private float clusterAngle;                  // Orientation of the bounding rectangle
  private float clusterSizeX, clusterSizeY;    // Dimensions of the bounding rectangle (along u and v)
  
  // Use Douglas Peucker algorithm to simplify the list of points that are assigned to this cluster, and build a bounding rectangle for the remaining points:
  public StaticCluster(ArrayList<PVector> points, float rectangleMargin) {
    //this.points = DouglasPeucker(points, 0, points.size()-1, DOUGLAS_PEUCKER_EPSILON);
    this.points = points;
    computeBoundingRectangle(rectangleMargin);
  }
  
  public void Draw() {
    color clusterColor = computeAvgColor();
    
    // Use the average color of the dynamic clusters inside this cluster to draw the bounding rectangle of the cluster:
    fill(clusterColor, 120);
    
    // Draw the bounding rectangle:
    pushMatrix();
    translate(centerX, centerY);
    rotate(clusterAngle);
    rect(0, 0, clusterSizeX, clusterSizeY);
    popMatrix();
    
    // Draw the lines inside the cluster:
    strokeWeight(4);
    stroke(clusterColor);
    PVector previous = points.get(0);
    for(int i = 1; i < points.size(); i++) {
      PVector current = points.get(i);
      line(previous.x, previous.y, current.x, current.y);
      previous = current;
    }
    strokeWeight(1);
  }
  
  private color computeAvgColor() {
    float sumR = 0, sumG = 0, sumB = 0;
    
    for(DynamicCluster cluster : matches) {
      sumR += red(cluster.clusterColor);
      sumG += green(cluster.clusterColor);
      sumB += blue(cluster.clusterColor);
    }
    
    int n = matches.size();
    return color(sumR / n, sumG / n, sumB / n);
  }
  
  private void computeBoundingRectangle(float margin) {
    int n = points.size();
    
    // Compute the average of the points:
    float avg_x = 0, avg_y = 0;
    for(PVector point : points) { avg_x += point.x; avg_y += point.y; }
    avg_x /= n; avg_y /= n;
    
    // Compute the covariances of the points:
    float cov_xx = 0, cov_xy = 0, cov_yy = 0;
    for(PVector point : points) {
      cov_xx += (point.x - avg_x) * (point.x - avg_x);
      cov_xy += (point.x - avg_x) * (point.y - avg_y);
      cov_yy += (point.y - avg_y) * (point.y - avg_y);
    }
    cov_xx /= n; cov_xy /= n; cov_yy /= n;
    
    // Compute the covariance matrix of the points, and use it to find their eigenvectors:
    SimpleMatrix cov = new SimpleMatrix(new double[][]{{cov_xx, cov_xy}, {cov_xy, cov_yy}});
    SimpleEVD evd = cov.eig();
    
    // Compute get the eigen vectors of the covariance matrix:
    SimpleBase v0 = evd.getEigenVector(0);
    SimpleBase v1 = evd.getEigenVector(1);
    
    // Convert the vectors into Processing's vectors:
    u = new PVector((float) v0.get(0, 0), (float) v0.get(1, 0)).normalize();
    v = new PVector((float) v1.get(0, 0), (float) v1.get(1, 0)).normalize();
    
    // Find the center of the rectangle, using the min and max projections values of the points along u and v:
    PVector current = points.get(0);
    
    float minU, maxU, minV, maxV;
    minU = maxU = current.dot(u);
    minV = maxV = current.dot(v);
    
    for(int i = 1; i < points.size(); i++) {
      current = points.get(i);
      float dotU = current.dot(u);
      float dotV = current.dot(v);
      
      if(dotU < minU) minU = dotU;
      else if(dotU > maxU) maxU = dotU;
      if(dotV < minV) minV = dotV;
      else if(dotV > maxV) maxV = dotV;
    }
    
    float centerU = (minU + maxU) / 2;
    float centerV = (minV + maxV) / 2;
    
    this.centerX = centerU * u.x + centerV * v.x;
    this.centerY = centerU * u.y + centerV * v.y;
    this.clusterAngle = atan2(u.y, u.x);
    
    this.clusterSizeX = margin + maxU - minU;
    this.clusterSizeY = margin + maxV - minV;
  }
  
  // Return if the point is inside the rectangle:
  public boolean contains(PVector point) {
    PVector delta = new PVector(point.x - centerX, point.y - centerY);
    float alongU = 2 * delta.dot(u);
    float alongV = 2 * delta.dot(v);
    
    return -clusterSizeX <= alongU && alongU <= clusterSizeX && -clusterSizeY <= alongV && alongV <= clusterSizeY;
  }
  
  // Run douglas peucker algorithm on the given list of points, between start and end (inclusives):
  private ArrayList<PVector> DouglasPeucker(ArrayList<PVector> points, int start, int end, float epsilon) {
    PVector u = PVector.sub(points.get(start), points.get(end), new PVector());
    u.normalize();
    
    // Find the point with maximum orthogonal distance:
    int index = -1;
    float maxDistance = 0;
    
    // Tempoary variables to compute the orthogonal distance:
    PVector v = new PVector();
    PVector v_along_u = new PVector();
    
    PVector startPoint = points.get(start);
    for(int i = start+1; i < end; i++) {
      PVector.sub(points.get(i), startPoint, v);
      PVector.mult(u, v.dot(u), v_along_u);
      float distance = PVector.sub(v, v_along_u).mag();
      
      if(distance >= maxDistance) {
        maxDistance = distance;
        index = i;
      }
    }    
    
    if(maxDistance > epsilon) {
      ArrayList<PVector> list1 = DouglasPeucker(points, start, index, epsilon);
      ArrayList<PVector> list2 = DouglasPeucker(points, index, end, epsilon);
      
      list1.remove(list1.size()-1);
      for(PVector point : list2) list1.add(point);
      
      return list1;
    }
    
    else {
      ArrayList<PVector> result = new ArrayList<PVector>();
      result.add(startPoint);
      result.add(points.get(end));
      return result;
    }
  }
  
  public void addDynamicClusterMatch(DynamicCluster cluster) {
    matches.add(cluster);
  }
  
  public boolean hasMatch() {
    return matches.size() > 0;
  }
  
  public DynamicCluster createAndMatchDynamicCluster(color clusterColor) {
    DynamicCluster cluster = new DynamicCluster(deepcopy(points), clusterColor);
    matches.add(cluster);
    
    return cluster;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Simple class to draw trajectories (list of points):
public class Trajectory {
  private PVector[] trajectory;
  private color trajectoryColor;
  
  private int startIndex;
  private int pointCount;
  
  public Trajectory(color trajectoryColor, int maxPoints) {
    this.trajectory = new PVector[maxPoints];
    this.trajectoryColor = trajectoryColor;
    this.startIndex = 0;
    this.pointCount = 0;
  }
  
  public void Draw() {
    stroke(0);
    fill(trajectoryColor);
    
    for(int i = 0; i < pointCount; i++) {
      PVector current = trajectory[(startIndex + i) % trajectory.length];
      
      ellipse(current.x, current.y, 10, 10);
    }
  }
  
  public void addPoint(PVector point) {
    int last = (startIndex + pointCount) % trajectory.length;
    
    trajectory[last] = point;
    
    if(pointCount < trajectory.length)
      pointCount++;
    else
      startIndex = (startIndex + 1) % trajectory.length;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// A dynamic cluster is used to track clusters between frames. At each frame, a dynamic cluster will use the points around it to update the its state
// estimate. Since this cluster only uses the points around it to update its position estimate, it's position is more
// likely to stay smooth between different frames, making it well suited to track objects between frames.
// Thus, a dynamic cluster also has a speed estimate, and keeps track of its past trajectory (for drawing):

public class DynamicCluster {
  public final color clusterColor;          // Color of the cluster
  
  private PVector position;                 // Current position of the cluster
  private PVector speed;                    // Speed of the cluster
  private ArrayList<PVector> shape;         // Current shape of the cluster
  
  private Trajectory trajectory;
  private StaticCluster currentMatch;       // Static cluster that represent this cluster on the current frame
  private int frameCountNoMatch = 0;        // Number of frames this cluster wasn't matched with a static cluster
  
  public DynamicCluster(ArrayList<PVector> shape, color clusterColor) {
    this.clusterColor = clusterColor;
    
    this.position = computeCenter(shape);
    this.speed = new PVector();
    this.shape = shape;
    
    this.trajectory = new Trajectory(clusterColor, 20);
    
    this.trajectory.addPoint(position);
  }
  
  // Update the dynamic cluster. If it's matched with a static cluster (so if the object is matched with a real object visible in the scene),
  // we can use this visible object to update our position estimate. Otherwise, we can use the speed estimate of the cluster to update where
  // the object is now (even if we can't see it):
  public void Update() {
    
    // Use the points inside the static cluster, and the shape matching algorithm to estimate the new position of this cluster:
    if(currentMatch != null) {
      int[] matching = computeMatching(shape, currentMatch.points);
      float[] transform = estimateTransform(shape, currentMatch.points, position, matching, 100);
      
      float dX = transform[0];
      float dY = transform[1];
      float dTheta = transform[2];
      
      // Basic filter for the speed:
      final float mem = 0.9f;
      speed.x = mem * speed.x + (1-mem) * dX;
      speed.y = mem * speed.y + (1-mem) * dY;
      
      // Update the position and orientation of the shape:
      transformShape(position, shape, dX, dY, dTheta);
      
      // Update the position of the cluster:
      position = new PVector(position.x + dX, position.y + dY);
    }
    
    // Else, use the cluster speed estimate to update it:
    else {
      transformShape(position, shape, speed.x, speed.y, 0);
      position = PVector.add(position, speed, new PVector());
    }
      
    // Add the position in the trajectory:
    trajectory.addPoint(position);
  }
  
  public void Draw() {
    trajectory.Draw();
    
    strokeWeight(4);
    drawShape(shape, clusterColor);
    strokeWeight(1);
    
    // Draw the speed estimate (with a scale factor, to avoid being too small):
    final float factor = 50;
    drawArrow(position.x, position.y, position.x + speed.x * factor, position.y + speed.y * factor, color(255, 0, 0));
  }
  
  public PVector position() {
    return position;
  }
  
  public void matchCluster(StaticCluster cluster) {
    this.currentMatch = cluster;
    
    // If the dynamic cluster wasn't matched with a static cluster during this frame, then the object this dynamic cluster is supposed to 
    // represent is not visible anymore. Maybe this is just an occlusion, and the object will appear again later. But if the cluster isn't 
    // matched with a real object for sufficiently many frames, it should be deleted:
    if(cluster == null)
      frameCountNoMatch++;
  }
  
  // Return the number of frames this cluster continued to exist without being matched with a real object in the scene.
  // If this number is big enough, this cluster should be deleted:
  public int getFrameCountNoMatch() {
    return frameCountNoMatch;
  }
}
