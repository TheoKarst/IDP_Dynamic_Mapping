import numpy as np

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
    Substracts the angles a and b (in radians) and keeps the result
    in the range ]-pi, pi]
    """

    # Compute a - b and keep the result in the range [0, 2*pi]:
    delta = (a - b) % (2 * np.pi)

    return np.where(delta > np.pi, delta - 2 * np.pi, delta)

def line_substract_angles(a : np.ndarray, b : np.ndarray):
    """ 
    Computes the smallest (in absolute value) difference between
    two lines with the given angles in radians. 
    The result is in the range ]-pi/2, pi/2]
    """

    num = (a - b) % np.pi

    return np.where(num > np.pi/2, num - np.pi, num)

def line_delta_angle(a : np.ndarray, b : np.ndarray):
    """ 
    Computes the smallest angle in radians between two lines.
    The result is in the range [0, pi/2]
    """

    num = (a - b) % np.pi

    return np.where(num > np.pi/2, np.pi - num, num)