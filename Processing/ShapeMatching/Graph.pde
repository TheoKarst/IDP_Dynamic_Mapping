class Graph {
  private float x, y;
  private float Width, Height;
  
  private color curveColor;
  
  private float[] curveValues;
  private float minValue = -PI, maxValue = PI;
  
  public final float margin = 20;
  
  
  public Graph(float x, float y, float Width, float Height, color curveColor) {
    this.x = x;
    this.y = y;
    this.Width = Width;
    this.Height = Height;
    
    this.curveColor = curveColor;
  }
  
  public void Draw() {
    noFill();
    stroke(0);
    rect(x + margin, y + margin, Width - 2*margin, Height -2*margin);
    
    if(curveValues == null)
      return;
      
    stroke(curveColor);
    float prevX = x + margin;
    float prevY = map(curveValues[0], minValue, maxValue, y + Height - margin, y + margin);
    
    for(int i = 1; i < curveValues.length; i++) {
      float pX = map(i, 0, curveValues.length-1, x + margin, x + Width - margin);
      float pY = map(curveValues[i], minValue, maxValue, y + Height - margin, y + margin);
      
      line(prevX, prevY, pX, pY);
      prevX = pX; prevY = pY;
    }
  }
  
  public void SetCurve(float[] curveValues) {
    this.curveValues = curveValues;
    
    /*
    if(curveValues != null) {
      minValue = maxValue = curveValues[0];
      for(int i = 1; i < curveValues.length; i++) {
        if(curveValues[i] < minValue) 
          minValue = curveValues[i];
        else if(curveValues[i] > maxValue) 
          maxValue = curveValues[i];
      }
    }*/
  }
}
  
