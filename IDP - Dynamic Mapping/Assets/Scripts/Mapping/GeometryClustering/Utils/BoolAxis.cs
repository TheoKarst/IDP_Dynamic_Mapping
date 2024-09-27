using System.Collections.Generic;
using System.Collections.ObjectModel;

/// <summary>
/// This class is used to represent a boolean function, able to map any real value
/// with a boolean.To do this, we store the state of the function at -infinity and
/// a list of splits at which the function switches from True to False or False to True
/// </summary>
public class BoolAxis {
    
    private bool defaultValue;
    private List<float> splits = new List<float>();

    /// <summary>
    /// Instantiates a BoolAxis with the given initial value
    /// </summary>
    public BoolAxis(bool defaultValue) {
        this.defaultValue = defaultValue;
    }

    /// <summary>
    /// Sets the value of the boolean function between min and max
    /// </summary>
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

    /// <summary>
    /// Returns the boolean value of the function at x
    /// </summary>
    public bool GetValue(float x) {

        int index = 0;
        while (index < splits.Count && splits[index] < x)
            index++;

        return index % 2 == 0 ? defaultValue : !defaultValue;
    }

    /// <summary>
    /// Resets the state of the axis
    /// </summary>
    /// <param name="defaultValue">Value of the axis at -infinity</param>
    /// <param name="changes">Sorted list of positions where the state of the axis should change between
    /// True and False or the opposite</param>
    public void Reset(bool defaultValue, List<float> changes) {
        this.defaultValue = defaultValue;
        this.splits = changes;
    }

    /// <summary>
    /// Returns the list of positions where the state of the function changes
    /// </summary>
    public ReadOnlyCollection<float> GetSplits() {
        return splits.AsReadOnly();
    }
}