class Cluster {
  private final ArrayList<PVector> points;
  private final color clusterColor;
  
  private float x, y;    // Center of the cluster
  private float clusterSizeX, clusterSizeY, clusterAngle;
  
  public Cluster(ArrayList<PVector> points, color clusterColor) {
    this.points = points;
    this.clusterColor = clusterColor;
  }
  
  public void Draw() {
    stroke(0);
    fill(clusterColor, 120);
    
    pushMatrix();
    translate(x, y);
    rotate(clusterAngle);
    rect(0, 0, clusterSizeX, clusterSizeY);
    popMatrix();
  }
  
  public void build() {
    int n = points.size();
    
    // Compute the average of the points:
    x = y = 0;
    for(PVector point : points) {
      x += point.x;
      y += point.y;
    }
    x /= n;
    y /= n;
    
    // Compute the covariances of the points:
    float cov_xx = 0, cov_xy = 0, cov_yy = 0;
    for(PVector point : points) {
      cov_xx += (point.x - x) * (point.x - x);
      cov_xy += (point.x - x) * (point.y - y);
      cov_yy += (point.y - y) * (point.y - y);
    }
    cov_xx /= n;
    cov_xy /= n;
    cov_yy /= n;
    
    // Compute the covariance matrix of the points, and use it to find their eigenvectors and eigenvalues:
    SimpleMatrix cov = new SimpleMatrix(new double[][]{{cov_xx, cov_xy}, {cov_xy, cov_yy}});
    SimpleEVD evd = cov.eig();
    
    SimpleBase v0 = evd.getEigenVector(0);
    SimpleBase v1 = evd.getEigenVector(1);
    
    // float e0 = (float) evd.getEigenvalue(0).real;
    // float e1 = (float) evd.getEigenvalue(1).real;
    
    PVector u = new PVector((float) v0.get(0, 0), (float) v0.get(1, 0)); //.mult(e0);
    PVector v = new PVector((float) v1.get(0, 0), (float) v1.get(1, 0)); //.mult(e1);
    
    float factorU = 0, factorV = 0;
    
    PVector tmp = new PVector();
    PVector center = new PVector(x, y);
    for(PVector point : points) {
      PVector.sub(point, center, tmp);
      float dotU = abs(tmp.dot(u));
      float dotV = abs(tmp.dot(v));
      
      if(dotU > factorU) factorU = dotU;
      if(dotV > factorV) factorV = dotV;
    }
    
    clusterAngle = atan2(u.y, u.x);
    clusterSizeX = 2 * factorU + 10;
    clusterSizeY = 2 * factorV + 10;
  }
}
