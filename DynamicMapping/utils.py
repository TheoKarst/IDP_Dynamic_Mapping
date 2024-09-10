import json
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
    Substract the angles a and b (in radians) and keeps the result
    in the range ]-pi, pi]
    """

    # Compute a - b and keep the result in the range [0, 2*pi]:
    delta = (a - b) % (2 * np.pi)

    return np.where(delta > np.pi, delta - 2 * np.pi, delta)

def load_json(path : str):
        """ Loads a json file and returns data as a dictionnary """

        with open(path, "r") as f:
            return json.load(f)
    