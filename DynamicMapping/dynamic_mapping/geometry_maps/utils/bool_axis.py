class BoolAxis:
    """
    This class is used to represent a boolean function, able to map any real value
    with a boolean. To do this, we store the state of the function at -infinity and
    a list of splits at which the function switches from True to False or False to True
    """

    def __init__(self, default_value):
        self.default_value = default_value
        self.splits = []

    def set_value(self, min : float, max : float, value : bool):
        """ Sets the value of the boolean function between min and max """

        if min >= max:
            return
        
        # If there is currently no split in the function, we have a constant function.
        if len(self.splits) == 0:

            # If the value is already the constant value of the function, there is notthing
            # to do. Otherwise, we just need to add two splits:
            if value != self.default_value:
                self.splits = [min, max]

            return
        
        # If the function is not constant, we have to update the splits:

        # Find the smallest index so that: min <= self.splits[start_index]:
        start_index = 0
        while start_index < len(self.splits) and self.splits[start_index] < min:
            start_index += 1

        # Find the biggest index so that: max < self.splits[end_index]:
        end_index = start_index
        while end_index < len(self.splits) and self.splits[end_index] <= max:
            end_index += 1

        # We need to keep the splits between 0 and start_index:
        new_splits = self.splits[:start_index]

        # Get the value of the section between start_index - 1 and start_index:
        current_value = self.default_value if start_index % 2 == 0 else not self.default_value

        # If the value is different from the new value we want to set, we add a split there:
        if value != current_value:
            new_splits.append(min)

        # Get the value of the section between end_index - 1 and end_index:
        current_value = self.default_value if end_index % 2 == 0 else not self.default_value

        # If the value is different from the new value we want to set, we add a split there:
        if value != current_value:
            new_splits.append(max)

        # Keep the splits from end_index to the end:
        new_splits += self.splits[end_index:]

        # Update the splits:
        self.splits = new_splits

    def get_value(self, x : float):
        """ Returns the value of the axis at x """

        index = 0
        while index < len(self.splits) and self.splits[index] < x:
            index += 1

        return self.default_value if index % 2 == 0 else not self.default_value
    
    def reset(self, default_value : bool, changes : list[float]):
        """ Resets the state of the axis
        
            :param default_value: Value of the axis at -infinity
            :param changes: Sorted list of positions where the state of the axis
                should change between True and False or the opposite
        """

        self.default_value = default_value
        self.splits = changes