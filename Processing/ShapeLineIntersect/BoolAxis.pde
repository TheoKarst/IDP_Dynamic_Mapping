public class BoolAxis {
  private boolean firstValue;
  private float rangeMin, rangeMax;
  private ArrayList<Float> splits = new ArrayList<Float>();
  
  public BoolAxis(float min, float max, boolean value) {
    this.rangeMin = min;
    this.rangeMax = max;
    this.firstValue = value;
  }
  
  public void SetValue_InPlace(float min, float max, boolean value) {
    // TODO: Implement this correctly...
    if(max <= min || max <= rangeMin || min >= rangeMax) return;
    
    int index = 0;
    while(index < splits.size() && splits.get(index) < min)
      index++;
    
    boolean currentValue = index % 2 == 0 ? firstValue : !firstValue;
    
    if(index == splits.size()) {
      if(value != currentValue) {
        if(min > rangeMin) splits.add(min); else firstValue = value;
        if(max < rangeMax) splits.add(max);
      }
      return;
    }
    
    float next = splits.get(index);
    if(value != currentValue) splits.set(index, min);
    
    while(next <= max && (++index < splits.size()))
      next = splits.get(index);
    
    currentValue = index % 2 == 0 ? firstValue : !firstValue;
  }
  
  public void SetValue(float min, float max, boolean value) {
    if(max <= min || max <= rangeMin || min >= rangeMax) return;
    
    int startIndex = 0;
    while(startIndex < splits.size() && splits.get(startIndex) < min)
      startIndex++;
    
    int endIndex = startIndex;
    while(endIndex < splits.size() && splits.get(endIndex) <= max)
      endIndex++;
      
    ArrayList<Float> newSplits = new ArrayList<Float>();
    for(int i = 0; i < startIndex; i++)
      newSplits.add(splits.get(i));
      
    boolean currentValue = startIndex % 2 == 0 ? firstValue : !firstValue;
    if(value != currentValue && min > rangeMin) newSplits.add(min);
    
    currentValue = endIndex % 2 == 0 ? firstValue : !firstValue;
    if(value != currentValue && max < rangeMax) newSplits.add(max);
    
    for(int i = endIndex; i < splits.size(); i++)
      newSplits.add(splits.get(i));
      
    if(min <= rangeMin) { rangeMin = min; firstValue = value; }
    if(max >= rangeMax) { rangeMax = max; }
    
    splits = newSplits;
  }
  
  @Override
  public String toString() {
    String out = "BoolAxis (first=" + firstValue + "): [" + rangeMin;
    for(float split : splits)
      out += "; " + split;
    out += "; " + rangeMax + "]";
    
    return out;
  }
}
