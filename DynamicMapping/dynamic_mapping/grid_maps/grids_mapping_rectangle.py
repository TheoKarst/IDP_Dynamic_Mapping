import numpy as np

import geometry_maps.utils.angles as utils

from scene.pose_2d import Pose2D
from grid_maps.grids_mapping import GridsMapping

# Implementation of the mapping with grid maps, using the paper "2D Occupancy Grid Mapping 
# With Inverse Range Sensor Model's algorithm" to project the rays of the LIDAR onto the grids
# (https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7244705).
# 
# The complexity of this algorithm is O(C*log(N)), where C is the number of cells and N is the number
# of rays of the LIDAR. Even if this is usually worse than for Bresenham's algorithm, this can be
# optimized using Numpy, which is finally much faster than the alternative.

class GridsMappingRectangle(GridsMapping):
    def __init__(self, center : tuple, width : int, height : int, cell_size : float,
                 alpha : float = None, beta : float = None):
        """ Creates an instance of grid mapping algorithm, able to represent static
        and dynamic objects in a scene using two maps. Numpy is used to project
        LIDAR observations onto the grids
        
            :param center: Tuple representing the center of the grid (in meters)
            :param width: Number of cells along the x-axis
            :param height: Number of cells along the y-axis
            :param cell_size: Size of a cell in meters
            :param alpha: Range error of a ray. If None, this is set to 2 * cell_size
            :param beta: Opening angle of a ray. If None, only the cells intersecting
                the ray will be updated (similarly to Bresenham's algorithm)
        """

        super().__init__(center, width, height, cell_size)

        # Parameters controlling the inverse sensor model, see figure 5 of
        # https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7244705:
        self.alpha = 2 * cell_size if alpha is None else alpha
        self.beta = beta
        
    def update_maps(self, world_sensor_pose : Pose2D, observations : dict):
        """
        Given the world pose of the sensor and the observations of that sensor, update
        the static and dynamic grids

            :param world_sensor_pose: Global pose of the sensor which made the observations
            :param observations: Observations made by that sensor
        """

        # Create a meshgrid to have the coordinates of each cell:
        X, Y = self.cell_to_world(np.arange(self.width), np.arange(self.height))
        Y, X = np.meshgrid(Y, X)

        # Compute the difference between the position of each cell and the sensor:
        dX, dY = X - world_sensor_pose.x, Y - world_sensor_pose.y
        
        # Compute the range and the angle between each cell and the sensor:
        ranges = np.sqrt(dX ** 2 + dY ** 2)
        angles = (np.arctan2(dY, dX) - world_sensor_pose.angle) % (2 * np.pi)

        # For each cell, find the index of the nearest observation from that cell:
        nearest_ray = self.find_nearest_ray(angles, observations)

        # Compute the angle between each cell and the nearest corresponding observation:
        angles = utils.abs_delta_angles(angles, observations['angles'][nearest_ray])

        # Compute which cells are seen by the LIDAR (traversed by a ray):
        nearest_range = observations['ranges'][nearest_ray]

        # If the opening angle is None, the max_angle is computed so that all the cells with an
        # orthogonal distance to the ray less than 'sqrt(2) * cell_size / 2' are considered to be
        # observed by the LIDAR. This means that a cell is considered to be observed if the 
        # circumcircle of the cell is collided by a ray:
        max_angle = np.sqrt(2) * self.cell_size / (2 * ranges) if self.beta is None else self.beta / 2
        observed = (ranges <= nearest_range + self.alpha / 2) & (angles <= max_angle)
        
        # Among the observed cells, compute which ones are occupied:
        occupied = ((np.abs(ranges - nearest_range) <= self.alpha / 2) \
                    & np.logical_not(observations['out_of_range'][nearest_ray]))[observed]
        
        # Compute for each observed cell if it's currently considered free from static objects:
        prev_static_free = self.log_odds_to_proba(self.static_map[observed]) <= 0.1

        # The occupancy probability in the static map should increase if the cell is observed
        # as occupied and was not free from static objects before:
        lo_static = np.where(occupied & np.logical_not(prev_static_free), 
                             self.lo_high_static, self.lo_low_static)

        # The occupancy probability in the dynamic map should increase if the cell is observed
        # as occupied and was free from static objects before:
        lo_dynamic = np.where(occupied & prev_static_free, 
                              self.lo_high_dynamic, self.lo_low_dynamic)

        # We can finally update the observed cells in the static and dynamic maps.
        # We clip the values to prevent the algorithm from being too confident about
        # the occupancy probability of the cells:
        self.static_map[observed] = np.clip(self.static_map[observed] + lo_static, 
                                            -self.max_log_odds, self.max_log_odds)
        
        self.dynamic_map[observed] = np.clip(self.dynamic_map[observed] + lo_dynamic, 
                                             -self.max_log_odds, self.max_log_odds)

    def find_nearest_ray(self, angles : np.ndarray, observations : dict):
        """ 
        Given the angles between each cell and the sensor, find for each cell the
        index of the closest observation to that cell

            :param angles: 2D array of containing for each cell the angle between
                that cell and the LIDAR
            :param observations: Observations made by the LIDAR, that must be sorted
                by increasing values of angles
            :returns: For each cell, the index of the closest ray from that cell
        """

        # Use searchsorted to find the two closest observations. For each cell,
        # we get the index of the first observation greater or equal to the
        # angle of the cell:
        right_indices = np.searchsorted(observations['angles'], angles)
        left_indices = right_indices - 1

        # Make sure to keep valid indices, knowing that angles are modulo two pi:
        right_indices %= len(observations['angles'])
        left_indices %= len(observations['angles'])

        # For each cell, get the angle to the two closest rays:
        left_values = utils.abs_delta_angles(angles, observations['angles'][left_indices])
        right_values = utils.abs_delta_angles(angles, observations['angles'][right_indices])
        
        return np.where(left_values < right_values, left_indices, right_indices)