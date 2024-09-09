import numpy as np
import utils
from pose_2d import Pose2D
# from scene import Scene

# Implementation of the mapping with grid maps, using the paper "2D Occupancy Grid Mapping 
# With Inverse Range Sensor Model's algorithm" to project the rays of the LIDAR onto the grids
# (https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7244705).
# 
# The complexity of this algorithm is O(C*log(N)), where C is the number of cells and N is the number
# of rays of the LIDAR. Even if this is usually worse than for Bresenham's algorithm, this can be
# optimized using Numpy, which is thus much faster than the alternative.
# 
# The world frame is defined with the x-axis pointing to the right and the y-axis pointing to
# the top. The representation for the grids uses the same representation: grid[x][y]

class GridsMappingSquare:
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

        self.center = center
        self.width = width
        self.height = height
        self.cell_size = cell_size

        # Parameters controlling the inverse sensor model, see figure 5 of
        # https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7244705:
        self.alpha = 2 * cell_size if alpha is None else alpha
        self.beta = beta

        # Create the static and dynamic maps. Since the maps are initially unknown, they should be
        # filled with zeros, because this is the log-odds form representation of a probability p=0.5:
        self.static_map = np.zeros((width, height))
        self.dynamic_map = np.zeros((width, height))

        # By default, the maximum confidence is set to one so the log-odds form is not clamped:
        self.max_log_odds = np.inf

        # Log-odds form of the parameters used in the inverse sensor model. Arbitrarly values are 
        # used, but can be setup correctly later:
        self.lo_low_static = self.proba_to_log_odds(0.4)
        self.lo_high_static = self.log_odds_to_proba(0.6)
        self.lo_low_dynamic = self.log_odds_to_proba(0.4)
        self.lo_high_dynamic = self.log_odds_to_proba(0.6)
        
    def setup_using_probas(self, low_static : float, high_static : float, low_dynamic : float, 
                           high_dynamic : float, max_confidence : float):
        """
        Setup the coefficients that are used to update the occupancy probability of a cell when a
        new observation is made
        """

        # Probabilities are clamped in the range [1-max_confidence; max_confidence], which
        # is equivalent to clamp log-odds values in the range [-max_log_odds; max_log_odds]:
        self.max_log_odds = self.proba_to_log_odds(max_confidence) if max_confidence < 1 else np.inf

        # Convert to log-odds form:
        self.lo_low_static = self.proba_to_log_odds(low_static)
        self.lo_high_static = self.proba_to_log_odds(high_static)
        self.lo_low_dynamic = self.proba_to_log_odds(low_dynamic)
        self.lo_high_dynamic = self.proba_to_log_odds(high_dynamic)

    def setup_using_frames_counts(self, frames_static : int, frames_dynamic : int, max_confidence : float):
        """
        Setup the coefficients that are used to update the occupancy probability of a cell when a
        new observation is made

            :param frames_static: Minimum number of frames to consider an object as static
            :param frames_dynamic: Minimum number of frames to consider an object as dynamic
            :param max_confidence: Value between 0.5 and 1 representing the maximum occupancy
                probability of a cell. The occupancy probability is clamped in the range
                [1-max_confidence; max_confidence]. Smaller values allows a faster update of
                the grid
        """

        # Probabilities are clamped in the range [1-max_confidence; max_confidence], which
        # is equivalent to clamp log-odds values in the range [-max_log_odds; max_log_odds]:
        self.max_log_odds = self.proba_to_log_odds(max_confidence) if max_confidence < 1 else np.inf

        # Compute the values for the inverse sensor model of the static map:
        self.lo_high_static = self.max_log_odds / frames_static
        self.lo_low_static = -self.lo_high_static

        # Compute the values for the inverse sensor model of the dynamic map:
        self.lo_high_dynamic = self.max_log_odds / frames_dynamic
        self.lo_low_dynamic = -self.lo_high_dynamic

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
        occupied = (np.abs(ranges - nearest_range) <= self.alpha / 2)[observed]
        
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
        Given the angles between each cel and the sensor, find for each cell the
        index of the closest observation to that cell

            :param angles: 2D array of containing for each cell the angle between
                that cell and the LIDAR
            :param observations: Observations made by the LIDAR
            :returns: For each cell, the index of the closest ray from that cell
        """

        # First, sort the observations by angle if necessary:
        utils.sort_observations(observations)

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

    def draw(self, scene : 'Scene'):
        """ Draws the static and dynamic maps on the scene """

        # Concatenate the static and dynamic maps:
        # data = np.concatenate((self.static_map, self.dynamic_map))
        data = np.flip(self.static_map, axis=1)

        # Convert the log-odds form into probabilities:
        data = self.log_odds_to_proba(data) < 0.9

        # Draw the grid in the scene:
        scene.draw_grid(self.center[0], self.center[1], self.cell_size, data)

    def world_to_cell(self, x : np.ndarray, y : np.ndarray):
        """ Converts a world position into a cell position 
        
            :param x: World position along the x-axis (in meters)
            :param y: World position along the y-axis (in meters)

            :returns: The position of the corresponding cell in the grids as a tuple
        """

        x = np.floor((x - self.center[0]) / self.cell_size + self.width / 2).astype(int)
        y = np.floor((y - self.center[1]) / self.cell_size + self.height / 2).astype(int)
        
        return x, y
    
    def cell_to_world(self, x : np.ndarray, y : np.ndarray):
        """ Converts a cell position into a world position 
        
            :param x: Index of a cell along the x-axis
            :param y; Index of a cell along the y-axis
            
            :returns: The world position in meters of the cell as a tuple (x, y)
        """

        x = self.center[0] + self.cell_size * (x + 0.5 - self.width / 2)
        y = self.center[1] + self.cell_size * (y + 0.5 - self.height / 2)

        return x, y
        

    def proba_to_log_odds(self, p : np.ndarray):
        """ Returns the log-odds form representation of a probability """

        return np.log(p / (1 - p))
    
    def log_odds_to_proba(self, l : np.ndarray):
        """ Returns the probability represented by the given log-odds form """

        e = np.exp(l)
        return e / (1 + e)