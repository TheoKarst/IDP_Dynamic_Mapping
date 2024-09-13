import math
from .dynamic_line import DynamicLine

class MatchGrid:
    """
    This class is used to project lines onto a grid in the mapping using geometric
    primitives. This allows to compute faster line matching
    """

    def __init__(self, width : int, height : int, cell_size : float, 
                 center_x : float = 0, center_y : float = 0):
        """ Instantiates a grid to speed up the matching of lines 
        
            :param width: Number of cells along the x-axis
            :param height: Number of cells along the y-axis
            :param cell_size: Size of a cell in meters
            :param center_x: World position of the grid center along the x-axis
            :param center_y: World position of the grid center along the y-axis
        """

        self.width = width
        self.height = height
        self.cell_size = cell_size

        # Position of the top-left corner of the grid:
        self.corner_x = center_x - width * cell_size / 2
        self.corner_y = center_y + height * cell_size / 2

        # Create an array of cells representing the grid, where each cell contains
        # the list of lines colliding that cell:
        self.cells = [[] for i in range(width * height)]

    def draw(self, scene : 'Scene'):
        """ Draws the grid in the scene """
        black = (0, 0, 0)

        start_x = self.corner_x
        end_x = self.corner_x + self.width * self.cell_size
        start_y = self.corner_y
        end_y = self.corner_y - self.height * self.cell_size
        
        # Draw the vertical lines:
        for i in range(self.width + 1):
            x = start_x + i * self.cell_size
            scene.draw_line(x, start_y, x, end_y, black)

        # Draw the horizontal lines:
        for i in range(self.height + 1):
            y = start_y - i * self.cell_size
            scene.draw_line(start_x, y, end_x, y, black)

        # Draw the occupied cell:
        for i in range(len(self.cells)):
            if self.cells[i]:   # If the cell is not empty
                x = self.corner_x + self.cell_size * ((i % self.width) + 0.5)
                y = self.corner_y - self.cell_size * ((i // self.width) + 0.5)

                scene.draw_rectangle(x, y, self.cell_size, self.cell_size, 0, black)

    def register_line(self, line : DynamicLine):
        # Compute the position of the endpoints of the line in the grid:
        x0, y0 = self.world_to_grid(line.begin_point[0], line.begin_point[1])
        x1, y1 = self.world_to_grid(line.end_point[0], line.end_point[1])

        if not self.belongs_to_grid(x0, y0) or not self.belongs_to_grid(x1, y1):
            print("Warning: The given line is outside of the bounds of the match grid !"\
                  "Make sure to use a large enough grid for the whole environment !")
            return

        cells = self.find_contact_cells(x0, y0, x1, y1)
        for x, y in cells:
            self.register_line_at(line, x, y)

    def register_line_at(self, line : DynamicLine, cell_x : int, cell_y : int):
        """ Adds the given line to the list of lines registered for the given cell """

        self.cells[self.width * cell_y + cell_x].append(line)

    def find_line_neighbors(self, line : DynamicLine):
        # Use a set to find neighbors, to avoid adding multiple times the same object in the list:
        neighbors = set()

        # Compute the position of the endpoints of the line in the grid:
        x0, y0 = self.world_to_grid(line.begin_point[0], line.begin_point[1])
        x1, y1 = self.world_to_grid(line.end_point[0], line.end_point[1])

        assert self.belongs_to_grid(x0, y0) and self.belongs_to_grid(x1, y1), \
            "The given line is outside of the bounds of the grid !"

        cells = self.find_contact_cells_margin(x0, y0, x1, y1)
        for x, y in cells:
            # Get the the lines registered for the cell (x, y):
            lines = self.cells[y * self.width + x]

            # Add the lines to the set:
            neighbors.update(lines)
        
        return neighbors

    def find_circle_neighbors(self, circle):
        """ Finds the lines in the same cell as the given circle """

        # Compute the position of the center of the circle in the grid:
        x, y = self.world_to_grid(circle.center[0], circle.center[1])
        x, y = math.floor(x), math.floor(y)

        assert self.belongs_to_grid(x, y), "The observed circle is outside of the grid !"

        return self.cells[y * self.width + x]

    def find_contact_cells(self, x0 : float, y0 : float, x1 : float, y1 : float):
        """
        Finds all the cells that are colliding the line with the given endpoints, using
        the voxel traversal algorithm described in the following paper:
        https://www.cse.chalmers.se/edu/year/2013/course/TDA361/grid.pdf

            :param x0: Smooth x-coordinate of the begin point of the line in the grid
                (between 0 and self.width)
            :param y0: Smooth y-coordinate of the begin point of the line in the grid
                (between 0 and self.height)
            :param x1: Smooth x-coordinate of the end point of the line in the grid
                (between 0 and self.width)
            :param y1: Smooth y-coordinate of the end point of the line in the grid
                (between 0 and self.height)
            :returns: A list of coordinates as tuples (x, y)
        """

        result : list[tuple[int, int]] = []

        x = math.floor(x0)
        y = math.floor(y0)
        x_end = math.floor(x1)
        y_end = math.floor(y1)

        step_x = 1 if x0 <= x1 else -1
        step_y = 1 if y0 <= y1 else -1

        if x == x_end:
            result.append((x, y))
            while y != y_end:
                y += step_y
                result.append((x, y))
            return result

        if y == y_end:
            result.append((x, y))
            while x != x_end:
                x += step_x
                result.append((x, y))
            return result

        t_max_x = (x + step_x - x0 if x0 < x1 else x - x0) / (x1 - x0)
        t_max_y = (y + step_y - y0 if y0 < y1 else y - y0) / (y1 - y0)

        slope_x = 1 / abs(x1 - x0)
        slope_y = 1 / abs(y1 - y0)

        steps_count = abs(x_end - x) + abs(y_end - y)
        for _ in range(steps_count + 1):
            result.append((x, y))

            if t_max_x < t_max_y:
                t_max_x += slope_x
                x += step_x
            else:
                t_max_y += slope_y
                y += step_y

        return result
    
    def find_contact_cells_margin(self, x0 : float, y0 : float, x1 : float, y1 : float):
        """
        Uses a modified version of Xiaolin Wu's algorithm, to get all the cells colliding the line
        with the given endpoints, returning all the points explored by the algorithm, no matter
        the brightness

            :param x0: Smooth x-coordinate of the begin point of the line in the grid
                (between 0 and self.width)
            :param y0: Smooth y-coordinate of the begin point of the line in the grid
                (between 0 and self.height)
            :param x1: Smooth x-coordinate of the end point of the line in the grid
                (between 0 and self.width)
            :param y1: Smooth y-coordinate of the end point of the line in the grid
                (between 0 and self.height)
            :returns: A list of coordinates as tuples (x, y)
        """

        result : list[tuple[int, int]] = []

        steep = abs(y1 - y0) > abs(x1 - x0)

        if steep:
            (x0, y0) = (y0, x0)
            (x1, y1) = (y1, x1)

        if x0 > x1:
            (x0, x1) = (x1, x0)
            (y0, y1) = (y1, y0)

        dx = x1 - x0
        dy = y1 - y0
        gradient = 1 if dx == 0 else dy / dx

        # Handle the first endpoint:
        xend = math.floor(x0)
        yend = y0 + gradient * (xend - x0) + (gradient - 1) / 2
        xpxl1 = xend    # This will be used in the main loop
        ypxl1 = math.floor(yend)
        if steep:
            result.append((ypxl1, xpxl1))
            result.append((ypxl1 + 1, xpxl1))
        else:
            result.append((xpxl1, ypxl1))
            result.append((xpxl1, ypxl1 + 1))

        # First y-intersection for the main loop:
        intery = yend + gradient

        # Handle second endpoint:
        xend = math.floor(x1)
        yend = y1 + gradient * (xend - x1) + (gradient - 1) / 2
        xpxl2 = xend    # This will be used in the main loop
        ypxl2 = math.floor(yend)
        if steep:
            result.append((ypxl2, xpxl2))
            result.append((ypxl2 + 1, xpxl2))
        else:
            result.append((xpxl2, ypxl2))
            result.append((xpxl2, ypxl2 + 1))

        # main loop
        if steep:
            for x in range(xpxl1 + 1, xpxl2):
                y = math.floor(intery)

                result.append((y, x))
                result.append((y + 1, x))
                intery = intery + gradient
        else:
            for x in range(xpxl1 + 1, xpxl2):
                y = math.floor(intery)

                result.append((x, y))
                result.append((x, y + 1))
                intery = intery + gradient

        return result

    def belongs_to_grid(self, x : float, y : float):
        """ Returns if the given coordinates belongs to the grid """
        return x >= 0 and x < self.width and y >= 0 and y < self.height
    
    def world_to_grid(self, x : float, y : float):
        """ Converts a coordinate from world space to grid space """

        x = (x - self.corner_x) / self.cell_size
        y = (self.corner_y - y) / self.cell_size

        return x, y
    
    def grid_to_world(self, x : int, y : int):
        """ Converts a coordinate from grid space to world space """

        x : float = self.corner_x + self.cell_size * (x + 0.5)
        y : float = self.corner_y - self.cell_size * (y + 0.5)

        return x, y
    
    def clear(self):
        """ Clears the cells of the grid from the registered lines """

        for i in range(len(self.cells)):
            self.cells[i].clear()