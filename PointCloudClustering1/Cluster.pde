///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// A rectangle cluster is used to represent a set of points that are near to each other.
// The rectangle cluster will use a bounding rectangle to represent the set of points it contains.
// The rectangle cluster is well suited to represent the current clusters on a frame (without taking into account the previous frames),
// and new rectangle clusters are therefore created each frame to match the current set of points
public class RectCluster {
  // List of point clusters that are inside this rectangle cluster:
  private ArrayList<PointCluster> matches = new ArrayList<PointCluster>();
  
  private final ArrayList<PVector> points;     // List of points that are contained in the rectangle cluster
  
  private PVector u, v;                        // Axis of the rectangle
  
  private float centerX, centerY;              // Center of the rectangle
  private float clusterAngle;                  // Orientation of the rectangle
  private float clusterSizeX, clusterSizeY;    // Dimensions of the rectangle (along u and v)
  
  public RectCluster(ArrayList<PVector> points, float rectangleMargin) {
    this.points = points;    
    computeBoundingRectangle(rectangleMargin);
  }
  
  public void Draw() {    
    stroke(0);
    
    // Use the average color of the point clusters inside this cluster to draw this rectangle cluster:
    fill(computeAvgColor(), 120);
    
    pushMatrix();
    translate(centerX, centerY);
    rotate(clusterAngle);
    rect(0, 0, clusterSizeX, clusterSizeY);
    popMatrix();
  }
  
  private color computeAvgColor() {
    float sumR = 0, sumG = 0, sumB = 0;
    
    for(PointCluster cluster : matches) {
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
  
  public void addPointClusterMatch(PointCluster cluster) {
    matches.add(cluster);
  }
  
  public boolean hasMatch() {
    return matches.size() > 0;
  }
  
  public PointCluster createAndMatchPointCluster(color clusterColor, float visibilityRadius) {
    PointCluster cluster = new PointCluster(centerX, centerY, clusterColor, visibilityRadius);
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

// A point cluster is used to represent the center of a cluster. At each frame, a point cluster will use the points around it to update the cluster's 
// center estimate, in a similar way to Kmeans. Since this cluster only uses the points around it to update its position estimate, it's position is more
// likely to stay smooth between different frames, making it well suited to track objects between frames.
// Thus, a point cluster also has a speed estimate, and keeps track of its past trajectory (for drawing):
public class PointCluster {
  private final float visibilityRadius;     // How far the point cluster "can see" in order to update its position
  private final color clusterColor;         // Color of the cluster
  
  private PVector position;                 // Current position of the cluster
  private PVector speed;                    // Speed of the cluster
  
  private Trajectory trajectory;
  private RectCluster currentMatch;         // Rectangle cluster that represent this cluster on the current frame
  private int frameCountNoMatch = 0;
  
  public PointCluster(float startX, float startY, color clusterColor, float visibilityRadius) {
    this.visibilityRadius = visibilityRadius;
    this.clusterColor = clusterColor;
    
    this.position = new PVector(startX, startY);
    this.speed = new PVector();    
    this.trajectory = new Trajectory(clusterColor, 20);
    
    this.trajectory.addPoint(position);
  }
  
  // Update the point cluster. If it's matched with a rectangle cluster (so if the object is matched with a real object visible in the scene),
  // we can use this visible object to update our position estimate. Otherwise, we can use the speed estimate of the cluster to update where
  // the object is now (even if we can't see it):
  public void Update() {
    
    // Use the mean of the points inside the visibility radius of the cluster to update the cluster position:
    if(currentMatch != null) {
      int count = 0;
      float avgX = 0, avgY = 0;
      
      for(PVector point : currentMatch.points) {
        if(PVector.dist(position, point) <= visibilityRadius) {
          avgX += point.x;
          avgY += point.y;
          count++;
        }
      }
      
      if(count != 0) {
        avgX /= count; avgY /= count;
        
        // Compute the difference between the previous position and the new one, to update the speed estimate of the cluster:
        float dX = avgX - position.x, dY = avgY - position.y;
        
        // Basic filter for the speed:
        final float mem = 0.9f;
        speed.x = mem * speed.x + (1-mem) * dX;
        speed.y = mem * speed.y + (1-mem) * dY;
        
        // Update the position of the cluster:
        position = new PVector(avgX, avgY);
      }
    }
    
    // Else, use the cluster speed estimate to update it:
    else
      position = PVector.add(position, speed, new PVector());
      
    // Add the position in the trajectory:
    trajectory.addPoint(position);
  }
  
  public void Draw() {
    trajectory.Draw();
    
    // Draw the speed estimate (with a scale factor, to avoid being too small):
    final float factor = 50;
    drawArrow(position.x, position.y, position.x + speed.x * factor, position.y + speed.y * factor, color(255, 0, 0));
  }
  
  public PVector position() {
    return position;
  }
  
  public void matchCluster(RectCluster cluster) {
    this.currentMatch = cluster;
    
    // If the point cluster wasn't matched with a rectangle cluster during this frame, then the object this point cluster is supposed to represent
    // is not visible anymore. Maybe this is just an occlusion, and the object will appear again later. But if the point cluster isn't matched
    // with a real object for sufficiently many frames, it should be deleted:
    if(cluster == null)
      frameCountNoMatch++;
  }
  
  // Return the number of frames this cluster continued to exist without being matched with a real object in the scene.
  // If this number is big enough, this cluster should be deleted:
  public int getFrameCountNoMatch() {
    return frameCountNoMatch;
  }
}
