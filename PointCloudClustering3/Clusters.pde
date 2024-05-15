final float MAX_MATCH_DISTANCE = 20;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class StaticCluster {
  private final ArrayList<PVector> points;
  private final boolean[] explained;
  
  public StaticCluster(ArrayList<PVector> points) {
    this.points = points;
    this.explained = new boolean[points.size()];    // For each point, if it can be explained by a dynamic cluster or not
  }
  
  public void Draw() {
    strokeWeight(1);
    PVector prev = points.get(0);
    
    stroke(0);
    fill(explained[0] ? color(0, 255, 0) : color(255, 0, 0));
    ellipse(prev.x, prev.y, 10, 10);
    
    for(int i = 1; i < points.size(); i++) {
      PVector current = points.get(i);
      
      // Draw circles for points:
      stroke(0);
      fill(explained[i] ? color(0, 255, 0) : color(255, 0, 0));
      ellipse(current.x, current.y, 10, 10);
    
      // Draw lines between points:
      stroke(0, 0, 255);
      line(prev.x, prev.y, current.x, current.y);
      prev = current;
    }
  }
  
  public void UpdatePointsExplanation(boolean[] explanations) {
    for(int i = 0; i < explained.length; i++)
      if(explanations[i])
        explained[i] = true;
  }
  
  public void BuildDynamicClusters(ArrayList<DynamicCluster> newClusters, int minPointsPerCluster) {
    ArrayList<PVector> current = new ArrayList<PVector>();
    for(int i = 0; i < points.size(); i++) {
      
      // For each point that is not explained, a new dynamic cluster has to be created:
      if(!explained[i]) {
        current.add(points.get(i));
      }
      
      else {
        if(current.size() >= minPointsPerCluster) {
          newClusters.add(new DynamicCluster(current));
          current = new ArrayList<PVector>();
        }
        else if(current.size() > 0)
          current.clear();
      }
    }
    
    if(current.size() >= minPointsPerCluster) {
      newClusters.add(new DynamicCluster(current));
      current = new ArrayList<PVector>();
    }
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

public class DynamicCluster {
  private color clusterColor;
  
  private PVector position;               // Current position of the cluster
  private PVector speed;                  // Speed of the cluster
  private Curve shape;                    // Current shape of the cluster
  
  private Trajectory trajectory;
  
  private StaticCluster currentMatch;       // To which static cluster this cluster is currently matched
  
  public DynamicCluster(ArrayList<PVector> shape) {
    this.clusterColor = randomColor();
    
    this.position = computeCenter(shape);
    this.speed = new PVector();
    this.shape = new Curve(shape, 20, 10);
    
    this.trajectory = new Trajectory(clusterColor, 20);
    this.trajectory.addPoint(position);
  }
  
  void MatchCluster(StaticCluster[] clusters) {
    int bestMatching[] = null;
    int bestScore = 0;
    currentMatch = null;
    
    for(StaticCluster cluster : clusters) {
      int matching[] = new int[shape.points.size()];
      int score = computeMatching(shape.points, cluster.points, matching);
      
      if(score > bestScore) {
        bestMatching = matching;
        bestScore = score;
        currentMatch = cluster;
      }
    }
    
    // If the cluster is matched with a static cluster points, we can update it using this static cluster:
    if(currentMatch != null) {
      float[] transform = estimateTransform(shape.points, currentMatch.points, position, bestMatching, 100);
      
      float dX = transform[0];
      float dY = transform[1];
      float dTheta = transform[2];
      
      // Basic filter for the speed:
      final float mem = 0.9f;
      speed.x = mem * speed.x + (1-mem) * dX;
      speed.y = mem * speed.y + (1-mem) * dY;
      
      // Update the position and orientation of the shape:
      transformShape(position, shape.points, dX, dY, dTheta);
      
      // Update the position of the cluster:
      position = new PVector(position.x + dX, position.y + dY);
    }
    
    // Else, update the cluster position just from its speed estimate:
    else {
      transformShape(position, shape.points, speed.x, speed.y, 0);
      position = PVector.add(position, speed, new PVector());
    }
  }
  
  // Try to explain how we could get the static shape matched with this cluster, by updating this dynamic clusters's contour estimate:
  public void ExplainStaticPoints() {
    // If we are currently not matched with a static cluster, there is nothing to explain (this dynamic cluster represents an object that
    // is not visible):
    if(currentMatch == null)
      return;
      
    boolean matches[] = shape.MatchCurve(currentMatch.points);    
    currentMatch.UpdatePointsExplanation(matches);
  }
  
  public void Draw() {
    trajectory.Draw();
    shape.Draw();
    // shape.DrawContour();
    
    // Draw the speed estimate (with a scale factor, to avoid being too small):
    final float factor = 50;
    drawArrow(position.x, position.y, position.x + speed.x * factor, position.y + speed.y * factor, color(255, 0, 0));
  }
  
  // initial and matching should have the same size
  // return the number of points from initial that were successfully matched
  private int computeMatching(ArrayList<PVector> initial, ArrayList<PVector> target, int matching[]) {
    int matchSuccess = 0;
    
    for(int i = 0; i < initial.size(); i++) {
      float minDistance = 0;
      int index = -1;
      
      for(int j = 0; j < target.size(); j++) {
        float distance = PVector.dist(initial.get(i), target.get(j));
        
        if(index == -1 || distance < minDistance) {
          index = j;
          minDistance = distance;
        }
      }
      
      if(minDistance <= MAX_MATCH_DISTANCE) {
        matching[i] = index;
        matchSuccess++;
      }
      else {
        matching[i] = -1;
      }
    }
    
    return matchSuccess;
  }
}
