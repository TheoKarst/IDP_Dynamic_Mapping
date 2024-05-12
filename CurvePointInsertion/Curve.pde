class Curve {
  private final color curveColor;
  private final ArrayList<PVector> points;
  
  public Curve(ArrayList<PVector> initialPoints, color curveColor) {
    this.curveColor = curveColor;
    this.points = initialPoints;
  }
  
  public void Draw() {
    stroke(curveColor);
    strokeWeight(2);
    
    PVector previous = points.get(0);
    for(int i = 1; i < points.size(); i++) {
      PVector current = points.get(i);
      line(previous.x, previous.y, current.x, current.y);
      previous = current;
    }
  }
  
  public void InsertPoint(PVector point) {
    
  }
}
