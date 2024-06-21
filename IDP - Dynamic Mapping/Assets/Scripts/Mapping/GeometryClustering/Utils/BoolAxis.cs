using System.Collections.Generic;

// Simple class to represent zones along an axis, between rangeMin and rangeMax.
// The class is used to map any value between rangeMin and rangeMax to a boolean.
// To to so, the axis is split in zones, starting with startValue (between rangeMin and split[0]),
// then not(startValue) (between split[0] and split[1]), etc...
// The state of the zones are thus changed for each split
public class BoolAxis {
    private bool startValue;
    private float rangeMin, rangeMax;
    private List<float> splits = new List<float>();

    public BoolAxis(float min, float max, bool value) {
        this.rangeMin = min;
        this.rangeMax = max;
        this.startValue = value;
    }

    // Set the value of the axis between min and max. The current range of the axis
    // and the given range should at least overlap for the changes to take effect
    public void SetValue(float min, float max, bool value) {
        // If the given range is wrong or doesn't overlap with the current axis,
        // we ignore it:
        if (max <= min || max <= rangeMin || min >= rangeMax)
            return;

        int startIndex = 0;
        while (startIndex < splits.Count && splits[startIndex] < min)
            startIndex++;

        int endIndex = startIndex;
        while (endIndex < splits.Count && splits[endIndex] <= max)
            endIndex++;

        List<float> newSplits = new List<float>();
        for (int i = 0; i < startIndex; i++)
            newSplits.Add(splits[i]);

        bool currentValue = startIndex % 2 == 0 ? startValue : !startValue;
        if (value != currentValue && min > rangeMin) newSplits.Add(min);

        currentValue = endIndex % 2 == 0 ? startValue : !startValue;
        if (value != currentValue && max < rangeMax) newSplits.Add(max);

        for (int i = endIndex; i < splits.Count; i++)
            newSplits.Add(splits[i]);

        // Extend the range of the axis if necessary:
        if (min <= rangeMin) { rangeMin = min; startValue = value; }
        if (max >= rangeMax) { rangeMax = max; }

        // Update the splits:
        splits = newSplits;
    }

    public void Reset(float min, float max, bool startValue, List<float> splits) {
        this.rangeMin = min;
        this.rangeMax = max;
        this.startValue = startValue;
        this.splits = splits;
    }

    public void RemoveGaps(float gapSize) {
        // TODO: Implement this !
    }

    public IEnumerable<(float min, float max)> GetTrueZones() {
        float prev = startValue ? rangeMin : splits[0];

        int index = startValue ? 0 : 1;
        for (; index < splits.Count; index += 2) {
            yield return (prev, splits[index]);
            if(index+1 < splits.Count) prev = splits[index+1];
        }

        if(index == splits.Count)
            yield return (prev, rangeMax);
    }

    public IEnumerable<(float min, float max)> GetFalseZones() {
        float prev = startValue ? splits[0] : rangeMin;

        int index = startValue ? 1 : 0;
        for (; index < splits.Count; index += 2) {
            yield return (prev, splits[index]);
            if(index+1 < splits.Count) prev = splits[index + 1];
        }

        if (index == splits.Count)
            yield return (prev, rangeMax);
    }

    public bool IsConstant() {
        return splits.Count == 0;
    }

    public bool StartValue() {
        return startValue;
    }

    public override string ToString() {
        return "[Start: " + startValue + ", Splits: [" + string.Join(", ", splits) + "]]";
    }
}