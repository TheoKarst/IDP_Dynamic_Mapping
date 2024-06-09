public class Line {
  private PVector start, end;
  private final PVector n;        // 90° rotation of (end - start) in counterclockwise order
  
  private boolean startValid = true;
  private ArrayList<PVector> points; 
  
  public Line(PVector start, PVector end) {
    this.start = start;
    this.end = end;
    this.n = new PVector(start.y - end.y, end.x - start.x);
    
    points = new ArrayList<PVector>();
    points.add(start);
    points.add(end);
  }
  
  public void Draw() {
    boolean valid = startValid;
    PVector prev = points.get(0);
    for(int i = 1; i < points.size(); i++) {
      PVector curr = points.get(i);
      
      stroke(valid ? color(0, 255, 0) : color(255, 0, 0));
      line(prev.x, -prev.y, curr.x, -curr.y);
      prev = curr;
      valid = !valid;
    }
    
    noStroke();
    fill(0);
    ellipse(start.x, -start.y, 5, 5);
  }
  
  public boolean IsLeftOfLine(PVector point) {
    return n.dot(PVector.sub(point, start, new PVector())) >= 0;
  }
  
  public float IntersectDistance(PVector A, PVector B) {
    PVector AB = PVector.sub(B, A, new PVector());
    PVector CD = PVector.sub(end, start, new PVector());
    
    float den = CD.x * AB.y - CD.y * AB.x;
    
    if(den == 0)
      return -1;
    
    PVector AC = PVector.sub(start, A, new PVector());
    
    return (AC.y * AB.x - AC.x * AB.y) / den;
  }
  
  public void UpdateState(ArrayList<Float> intersections, boolean startValid) {
    println("Update state");
    
    this.startValid = startValid;
    
    PVector u = PVector.sub(end, start, new PVector());
    
    points = new ArrayList<PVector>();
    points.add(start);
    
    for(float distance : intersections)
      points.add(PVector.mult(u, distance, new PVector()).add(start));
      
    points.add(end);
  }
}
