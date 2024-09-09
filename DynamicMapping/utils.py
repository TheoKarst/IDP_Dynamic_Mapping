import numpy as np

def sort_observations(observations : dict):
    """ Sort the given observations by increasing values of angle """

    # If the observations are already sorted, then return:
    if observations.get('sorted', False):
        return
    
    indices = np.argsort(observations['angles'])

    observations['ranges'] = observations['ranges'][indices]
    observations['angles'] = observations['angles'][indices]
    observations['sorted'] = True

def abs_delta_angles(a : np.ndarray, b : np.ndarray):
    """
    Computes the absolute difference between two angles in radians and 
    keeps the result in the range [0, pi]
    """

    # Compute a - b and keep the result in the range [0, 2*pi]:
    delta = (a - b) % (2 * np.pi)

    return np.where(delta > np.pi, 2 * np.pi - delta, delta)

def substract_angles(a : np.ndarray, b : np.ndarray):
    """ 
    Substract the angles a and b (in radians) and keeps the result
    in the range ]-pi, pi]
    """

    # Compute a - b and keep the result in the range [0, 2*pi]:
    delta = (a - b) % (2 * np.pi)

    return np.where(delta > np.pi, delta - 2 * np.pi, delta)
    