# File containing all the parameters that can be tuned in this project

import numpy as np
import utils

class RobotParams:

    # Estimated errors on the pose of the robot. This will be used by default
    # if not defined in the recorded data:
    ERROR_X = 0.1
    ERROR_Y = 0.1
    ERROR_ROBOT_ANGLE = 1 * np.pi / 180

    # Estimated errors on the observations of the LIDAR. This will be used by
    # default if not defined in the recorded data:
    ERROR_RANGE = 0.1
    ERROR_OBSERVATION_ANGLE = 1 * np.pi / 180

    def state_covariance():
        return utils.cov_matrix([
            RobotParams.ERROR_X, RobotParams.ERROR_Y, RobotParams.ERROR_ROBOT_ANGLE])
    
    def observation_covariance():
        return utils.cov_matrix([
            RobotParams.ERROR_RANGE, RobotParams.ERROR_OBSERVATION_ANGLE
        ])
    
def load_geometry_mapping_parameters():

    # TODO: Correctly implement this:
    parameters = {}
    parameters['points_critical_distance'] = 1
    parameters['line_critical_distance'] = 1
    parameters['points_critical_angle'] = 1
    parameters['line_min_points'] = 3
    parameters['line_min_length'] = 1
    parameters['line_process_noise_error'] = np.identity(2)
    parameters['circle_critical_distance'] = 1
    parameters['circle_min_points'] = 1

    return parameters