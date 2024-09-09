from math import cos, sin
from pose_2d import Pose2D
import numpy as np

def test(robot, lidar_local, observations):

        costheta = cos(robot.angle)
        sintheta = sin(robot.angle)

        # Compute the global angle of the observation:
        theta = robot.angle + lidar_local.angle + observations['angles']

        X = robot.x + lidar_local.x * costheta - lidar_local.y * sintheta + observations['ranges'] * np.cos(theta)
        Y = robot.y + lidar_local.x * sintheta + lidar_local.y * costheta + observations['ranges'] * np.sin(theta)

        return X, Y