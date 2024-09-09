import numpy as np
# from scene import Scene

# Base class for the grids mapping algorithms.
# 
# The world frame is defined with the x-axis pointing to the right and the y-axis pointing to
# the top. The representation for the grids uses the same representation: grid[x][y]

class GridsMapping:
    def __init__(self, center : tuple, width : int, height : int, cell_size : float):
        """ Base class to implement grid mapping algorithms, able to represent static
        and dynamic objects in a scene using two maps.
        
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

    def draw(self, scene : 'Scene', view : str = 'both', threshold : float | None = None):
        """ 
        Draws the static and dynamic maps on the scene
        
            :param scene: The scene in which the map should be drawn
            :param view: 'static' to draw only the static map, 'dynamic' to draw only
                the dynamic map or 'both' to draw both maps
            :param threshold: If None, the occupancy probability of each cell is
                represented using grayscales, otherwise cells with an occupancy
                probability above the threshold are drawn in black and other cells
                are drawn in white
        """

        if view == 'static':
            data = self.static_map
        elif view == 'dynamic':
            data = self.dynamic_map
        else:
            data = np.concatenate((self.static_map, self.dynamic_map))

        # Filp the data to draw the grid in the right order:
        data = np.flip(data, axis=1)

        # Convert the log-odds form into probabilities:
        data = self.log_odds_to_proba(data)

        # Convert probabilities into grayscales:
        if threshold is None:
            data = 1 - data             # Occupied cells should be black
        else:
            data = data < threshold     # Only show occupied cells

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