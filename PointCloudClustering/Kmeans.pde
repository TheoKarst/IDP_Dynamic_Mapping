class Trajectory {
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
  
  public PVector getCurrentPosition() {
    return trajectory[(startIndex + pointCount - 1) % trajectory.length];
  }
}

class Kmeans {
  private final float radius;
  private Trajectory[] clusterTrajectories;
  
  public Kmeans(float radius) {
    this.radius = radius;
  }
  
  public void setupClusters(ArrayList<Cluster> clusters) {
    clusterTrajectories = new Trajectory[clusters.size()];
    
    for(int i = 0; i < clusters.size(); i++) {
      clusterTrajectories[i] = new Trajectory(clusters.get(i).clusterColor, 100);
      clusterTrajectories[i].addPoint(clusters.get(i).getPosition());
    }
  }
  
  // Update each cluster using a "pseudo" Kmeans algorithm:
  public void UpdateClusters(PVector[] points) {
    for(Trajectory trajectory : clusterTrajectories) {
      PVector currentPosition = trajectory.getCurrentPosition();
      
      // Find the neighbours of the current position, and get their center:
      int neighboursCount = 0;
      float sumX = 0, sumY = 0;
      for(PVector point : points) {
        if(PVector.dist(currentPosition, point) <= radius) {
          sumX += point.x;
          sumY += point.y;
          neighboursCount++;
        }
      }
      
      // Use the center of the neighbours to update the current cluster position:
      if(neighboursCount > 0)
        trajectory.addPoint(new PVector(sumX / neighboursCount, sumY / neighboursCount));
    }
  }
  
  public void Draw() {
    for(Trajectory trajectory : clusterTrajectories)
      trajectory.Draw();
  }
}
