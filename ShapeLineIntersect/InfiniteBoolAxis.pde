public class InfiniteBoolAxis {
    private boolean defaultValue;
    private ArrayList<Float> splits = new ArrayList<Float>();

    public InfiniteBoolAxis(boolean defaultValue) {
        this.defaultValue = defaultValue;
    }
    
    public boolean GetValue(float x) {
      int index = 0;
      while(index < splits.size() && splits.get(index) < x)
        index++;
        
      return index % 2 == 0 ? defaultValue : !defaultValue;
    }

    public void SetValue(float min, float max, boolean value) {
      if(min >= max) return;
      
        if(splits.size() == 0) {
            if(value != defaultValue) {
                splits.add(min); 
                splits.add(max);
            }

            return;
        }

        int startIndex = 0;
        while (startIndex < splits.size() && splits.get(startIndex) < min)
            startIndex++;

        int endIndex = startIndex;
        while (endIndex < splits.size() && splits.get(endIndex) <= max)
            endIndex++;

        ArrayList<Float> newSplits = new ArrayList<Float>();
        for (int i = 0; i < startIndex; i++)
            newSplits.add(splits.get(i));

        boolean currentValue = startIndex % 2 == 0 ? defaultValue : !defaultValue;
        if (value != currentValue) newSplits.add(min);

        currentValue = endIndex % 2 == 0 ? defaultValue : !defaultValue;
        if (value != currentValue) newSplits.add(max);

        for (int i = endIndex; i < splits.size(); i++)
            newSplits.add(splits.get(i));

        // Update the splits:
        splits = newSplits;
    }
}
