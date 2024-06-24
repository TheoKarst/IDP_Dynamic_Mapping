using System.Collections.Generic;

public class BoolAxis2 {
    private bool defaultValue;
    private List<float> splits = new List<float>();

    public BoolAxis2(bool defaultValue) {
        this.defaultValue = defaultValue;
    }

    public void SetValue(float min, float max, bool value) {
        if (max <= min)
            return;

        if(splits.Count == 0) {
            if(value != defaultValue) {
                splits.Add(min); 
                splits.Add(max);
            }

            return;
        }

        int startIndex = 0;
        while (startIndex < splits.Count && splits[startIndex] < min)
            startIndex++;

        int endIndex = startIndex;
        while (endIndex < splits.Count && splits[endIndex] <= max)
            endIndex++;

        List<float> newSplits = new List<float>();
        for (int i = 0; i < startIndex; i++)
            newSplits.Add(splits[i]);

        bool currentValue = startIndex % 2 == 0 ? defaultValue : !defaultValue;
        if (value != currentValue) newSplits.Add(min);

        currentValue = endIndex % 2 == 0 ? defaultValue : !defaultValue;
        if (value != currentValue) newSplits.Add(max);

        for (int i = endIndex; i < splits.Count; i++)
            newSplits.Add(splits[i]);

        // Update the splits:
        splits = newSplits;
    }
}