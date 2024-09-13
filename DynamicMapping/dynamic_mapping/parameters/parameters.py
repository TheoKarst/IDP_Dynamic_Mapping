import numpy as np

import data.utils as utils

def convert_to_radians(dict : dict, key : str):
    """ 
    Converts the value with the given key in the given dictionnary 
    from degrees to radians
    """

    dict[key] = np.deg2rad(dict[key])

def cov_matrix(errors_dict : dict):
    """
    Creates a diagonal matrix where the elements on the diagonal
    are the quares of the values in the dictionnary
    """

    return np.diag(np.fromiter(errors_dict.values(), dtype=float) ** 2)

# Load the robot parameters and convert degrees to radians:
robot_params = utils.load_json('dynamic_mapping/parameters/robot_params.json')
convert_to_radians(robot_params['robot_pose'], 'error_angle')
convert_to_radians(robot_params['observation'], 'error_angle')

# Compute the default state covariance and observation covariance matrices:
state_covariance = cov_matrix(robot_params['robot_pose'])
observation_covariance = cov_matrix(robot_params['observation'])

# Load parameters for the mapping using geometric primitives and convert degrees to radians:
geometry_params = utils.load_json('dynamic_mapping/parameters/geometry_map_params.json')
convert_to_radians(geometry_params['geometry_extraction'], 'points_critical_angle')
convert_to_radians(geometry_params['geometry_matching'], 'line_max_match_angle')
convert_to_radians(geometry_params['dynamic_lines'], 'lines_max_angle_error')
convert_to_radians(geometry_params['dynamic_lines'], 'static_max_angle_derivative')
convert_to_radians(geometry_params['dynamic_lines']['lines_process_noise'], 'angle')
convert_to_radians(geometry_params['dynamic_lines']['lines_process_noise'], 'der_angle')
convert_to_radians(geometry_params['wipe_shape'], 'alpha')

# Compute the line process noise error matrix:
line_process_noise_error = cov_matrix(geometry_params['dynamic_lines']['lines_process_noise'])
