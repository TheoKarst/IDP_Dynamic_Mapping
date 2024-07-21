public class Observation {
  public final float r;
  public final float theta;
  
  public final float x;
  public final float y;
  
  public Observation(float r, float theta) {
    this.r = r;
    this.theta = theta;
    
    x = lidarCenter.x + r * cos(theta);
    y = lidarCenter.y + r * sin(theta);
  }
}
