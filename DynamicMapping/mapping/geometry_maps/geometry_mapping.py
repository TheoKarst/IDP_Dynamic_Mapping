import numpy as np
from robot import Robot

class GeometryMapping:
    def __init__(self):
        self.last_time_update = -1

    def update(self, robot : Robot, lidar_index : int, observations : dict, current_time : float):

        # Compute the elapsed time since the last update:
        delta_time = 0 if self.last_time_update == -1 else current_time - self.last_time_update
        self.last_time_update = current_time

        # Compute the position and error estimate of each observation:
        points = self.compute_points(robot, lidar_index, observations)

    def compute_points(self, robot : Robot, lidar_index : int, observations : dict):

        # Compute the position of the observations in world space:
        positions = robot.get_observations_positions(lidar_index)

        # Compute the covariance matrices representing the error estimates on these
        # positions:
        covariances = robot.get_observations_positions_covariances(lidar_index)
        
        # Create a dictionnary to represent points and remove observations that are out
        # of the range of the LIDAR:
        keep_indices = np.logical_not(observations['out_of_range'])

        points = {}
        points['positions'] = positions[keep_indices]
        points['covariances'] = covariances[keep_indices]

        return points