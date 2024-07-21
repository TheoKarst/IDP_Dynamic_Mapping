class PointCloud {
  private color cloudColor;
  private float pointSize;
  
  private int currentIndex = 0;
  
  private double[] time;
  private PVector[] positions;
  
  private ArrayList<PVector> currentFrame = null;
  
  public PointCloud(float pointSize, color cloudColor) {
    this.pointSize = pointSize;
    this.cloudColor = cloudColor;
  }
  
  public void loadData(String filename) {
    String[] lines = loadStrings(filename);
  
    this.time = new double[lines.length];
    this.positions = new PVector[lines.length];
    
    for(int i = 0; i < lines.length; i++) {
      String[] data = lines[i].split(" ");
      
      time[i] = Double.valueOf(data[0]);
      positions[i] = new PVector(float(data[1]), float(data[2]), float(data[3]));
    }
  }
  
  public void nextFrame(float timeInterval) {
    if(time == null || currentIndex >= time.length) {
      println("Last frame of the data");
      return;
    }
      
    currentFrame = new ArrayList<PVector>();
    
    println("Start time: " + time[currentIndex]);
    double stopTime = time[currentIndex] + timeInterval;
    
    // Collect all the points we have during the given time interval:
    do {
      currentFrame.add(positions[currentIndex]);
      currentIndex++;
    }
    while(currentIndex < time.length && time[currentIndex] <= stopTime);
    
    println("Frame size: " + currentFrame.size());
  }
  
  public void Draw() {
    if(currentFrame == null)
      return;
      
    final float scaleFactor = 100;
      
    sphereDetail(3);
    fill(cloudColor);
    for(PVector point : currentFrame) {
      // point(scaleFactor*point.x, scaleFactor*point.y, scaleFactor*point.z);
      translate(scaleFactor*point.x, scaleFactor*point.y, scaleFactor*point.z);
      sphere(pointSize);
      translate(-scaleFactor*point.x, -scaleFactor*point.y, -scaleFactor*point.z);
    }
  }
}
