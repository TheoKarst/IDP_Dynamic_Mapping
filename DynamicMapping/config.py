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