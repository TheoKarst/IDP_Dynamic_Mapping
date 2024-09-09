import numpy as np
from pose_2d import Pose2D
# from scene import Scene

# Implementation of the mapping with grid maps, using Bresenham's algorithm to project the
# rays of the LIDAR onto the grids. The compexity of this algorithm is O(N*M), where N
# is the number of rays of the LIDAR and M is the maximum range of the LIDAR.

# The world frame is defined with the x-axis pointing to the right and the y-axis pointing to
# the top. The representation for the grids uses the same representation: grid[x][y]

class GridsMappingBresenham:
    def __init__(self, center : tuple, width : int, height : int, cell_size : float):
        """ Creates an instance of grid mapping algorithm, able to represent static
        and dynamic objects in a scene using two maps. Bresenham's algorithm is used
        to project LIDAR observations onto the grids
        
            :param center: Tuple representing the center of the grid (in meters)
            :param width: Number of cells along the x-axis
            :param height: Number of cells along the y-axis
            :param cell_size: Size of a cell in meters    
        """

        self.center = center
        self.width = width
        self.height = height
        self.cell_size = cell_size

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

        # Compute the position of the sensor in the grids:
        x0, y0 = self.world_to_cell(world_sensor_pose.x, world_sensor_pose.y)

        # Compute the end position of all the observations:
        angles = world_sensor_pose.angle + observations['angles']

        x_world = world_sensor_pose.x + observations['ranges'] * np.cos(angles)
        y_world = world_sensor_pose.y + observations['ranges'] * np.sin(angles)

        # Convert the world positions into cell positions:
        x1, y1 = self.world_to_cell(x_world, y_world)

        # For each observation, use Bresenham's algorithm to project the ray onto the grids,
        # detect which cells are currently free or occupied and update the maps:
        for x, y in zip(x1, y1):
            self.bresenham(x0, y0, x, y, True)

    def bresenham(self, x0 : int, y0 : int, x1 : int, y1 : int, collision : bool):
        """
        Uses Bresenham's algorithm to update the cells traversed by the given line

            :param x0: x-position of the begin point of the line in the grid
            :param y0: y-position of the begin point of the line in the grid
            :param x1: x-position of the endpoint of the line in the grid
            :param y1: y-position of the endpoint of the line in the grid
            :param collision: If true, the last cell is considered occupied, otherwise it's
                considered free
        """

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = (dx if dx > dy else -dy) / 2

        while True:
            if x0 == x1 and y0 == y1:
                # If there was a collision, the last cell touched by the raycast is occupied.
                # Else, it's free:
                self.update_cell(x0, y0, collision)
                break

            # All the cells traversed by the raycast are free:
            self.update_cell(x0, y0, False)

            e2 = err
            if e2 > -dx:
                err -= dy
                x0 += sx

            if e2 < dy:
                err += dx
                y0 += sy

    def update_cell(self, x : int, y : int, occupied : bool):
        """ 
        Update the state of a cell in the static and dynamic maps, depending on if it's seen
        as free or occupied
        
            :param x: x-position of the cell to update
            :param y: y-position of the cell to update
            :param occupied: If the cell is seen as free or occupied
        """

        # If the cell is out of the grid, return:
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return
        
        # Check if the cell is currently considered as free from static objects:
        prev_static_free = self.log_odds_to_proba(self.static_map[x][y]) <= 0.1

        # The occupancy probability in the static map should increase if the cell is observed
        # as occupied and was not free from static objects before:
        lo_static = self.lo_high_static if occupied and not prev_static_free else self.lo_low_static

        # The occupancy probability in the dynamic map should increase if the cell is observed
        # as occupied and was free from static objects before:
        lo_dynamic = self.lo_high_dynamic if occupied and prev_static_free else self.lo_low_dynamic

        # Update the maps using lo_static and lo_dynamic (we are using a log-odds form representation)
        # and clip the values to avoid being too confident about the presence or absence of an
        # obstacle (this allows to be more reactive to changes in the environment):
        self.static_map[x][y] = np.clip(self.static_map[x][y] + lo_static, 
                                        -self.max_log_odds, self.max_log_odds)
        
        self.dynamic_map[x][y] = np.clip(self.dynamic_map[x][y] + lo_dynamic, 
                                        -self.max_log_odds, self.max_log_odds)

    def draw(self, scene : 'Scene'):
        # Concatenate the static and dynamic maps:
        # data = np.concatenate((self.static_map, self.dynamic_map))
        data = np.flip(self.static_map, axis=1)
        # Convert the log-odds form into probabilities:
        data = self.log_odds_to_proba(data)

        # Draw the grid in the scene:
        scene.draw_grid(self.center[0], self.center[1], self.cell_size, 1 - data)

    def world_to_cell(self, x : np.ndarray, y : np.ndarray):
        """ Convert a world position into a cell position 
        
            :param x: World position along the x-axis (in meters)
            :param y: World position along the y-axis (in meters)

            :returns: The position of the corresponding cell in the grids as a tuple
        """

        x = np.floor((x - self.center[0]) / self.cell_size + self.width / 2).astype(int)
        y = np.floor((y - self.center[1]) / self.cell_size + self.height / 2).astype(int)
        
        return x, y

    def proba_to_log_odds(self, p : np.ndarray):
        """ Returns the log-odds form representation of a probability """

        return np.log(p / (1 - p))
    
    def log_odds_to_proba(self, l : np.ndarray):
        """ Returns the probability represented by the given log-odds form """

        e = np.exp(l)
        return e / (1 + e)
