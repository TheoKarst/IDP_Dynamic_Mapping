import math
import pygame as py
import numpy as np

from scene.robot import Robot
from data.dataloader import Dataloader
from grid_maps.grids_mapping_bresenham import GridsMappingBresenham
from grid_maps.grids_mapping_rectangle import GridsMappingRectangle
from geometry_maps.geometry_mapping import GeometryMapping

class Scene:
    def __init__(self, screen : py.Surface, center_x : float, center_y : float, pixels_per_meter : float,
                 data_folder : str, use_grids_map : bool = True, use_geometry_map : bool = True):
        """
        Creates a scene for the mapping simulation using grid maps and geometric primitives
        
            :param screen: Screen on which the scene should be drawn
            :param center_x: Center of the camera along the x-axis
            :param center_y: Center of the camera along the y-axis
            :param pixels_per_meter: Number of pixels on the screen to represent one meter in the world
            :param data_folder: Folder from which the recorded data should be read
            :param use_grids_map: If the mapping using grid maps is toggled
            :param use_geometry_maps: If the mapping using geometry maps is toggled
        """
        
        self.screen = screen

        # World position of the top left corner of the camera:
        self.corner_x = center_x - screen.get_width() / (2 * pixels_per_meter)
        self.corner_y = center_y + screen.get_height() / (2 * pixels_per_meter)

        self.pixels_per_meter = pixels_per_meter

        # Text font to draw the current FPS on the screen:
        self.text_font = py.font.SysFont('Comic Sans MS', 14)

        # Create a dataloader to load recorded data:
        self.dataloader = Dataloader(data_folder)

        # Instantiate a robot with the setup defined in the recorded data:
        robot_setup = self.dataloader.load_robot_setup()
        self.robot = Robot(**robot_setup)

        # Instantiate a manager for grid mapping:
        if use_grids_map:
            self.grid_maps = GridsMappingRectangle((0,0), 700, 400, 0.1)
            self.grid_maps.setup_using_frames_counts(60, 5, 0.999)
        else:
            self.grid_maps = None

        # Instantiate a manager for mapping using geometric primitives:
        if use_geometry_map:
            self.geometry_map = GeometryMapping()
        else:
            self.geometry_map = None

        # Record what to display in the scene using a dictionnary:
        self.display = {}
        self.display['rays'] = False                # Draw the rays of the LIDARs
        self.display['grid_maps'] = 0               # 0: none, 1: static, 2: dynamic, 3: both
        self.display['geometry_map'] = True
        self.display['wipe_shape'] = True           # Draw the wipe shape
        self.display['match_grid'] = False          # Draw the match grid used for geometry mapping
        self.display['speed_estimates'] = True      # Draw the speed estimate of the primitives
        self.display['lines'] = True                # Draw the lines in the geometry map
        self.display['circles'] = True              # Draw the circles in the geometry map

        # For information, print the boundaries of the screen in world space:
        xmin, ymin = self.world_coordinates(0, 0)
        xmax, ymax = self.world_coordinates(screen.get_width(), screen.get_height())
        print(f"Screen setup: [xmin: {xmin}m; xmax: {xmax}m], [ymin: {ymin}m; ymax: {ymax}m]")

        # Last time when the scene was updated:
        self.last_time_update = 0

    def update(self, frame_mode : bool = False):
        """
        Called each frame, updates the whole scene
        
            :param frame_mode: If True, the data is loaded frame by frame instead of
                trying to run realtime
        """

        # Get the current time (number of seconds since pygame.init() was called):
        current_time = py.time.get_ticks() / 1000

        # Compute the delta time since the last update and load the next frame:
        if frame_mode:
            delta_time = 0.02
            self.last_time_update += 0.02

            data_frame = self.dataloader.load_next_frame()
        else:
            delta_time = current_time - self.last_time_update
            self.last_time_update = current_time

            data_frame = self.dataloader.load_current_frame(current_time)

        if data_frame is not None:
            # The first thing to update is the robot, to have the right position for the 
            # robot, the LIDARs and the observations during the frame:
            self.robot.update(**data_frame)

            # For each LIDAR, use the observations of that LIDAR to update the world models:
            for lidar_index in range(len(self.robot.lidars)):

                # Get the global pose of the LIDAR:
                lidar_pose = self.robot.lidars[lidar_index].global_pose

                # Get the observations of that LIDAR:
                observations = self.robot.lidars[lidar_index].observations

                # Use the observations of the LIDAR to update the static and dynamic maps:
                if self.grid_maps is not None:
                    self.grid_maps.update_maps(lidar_pose, observations)

                # Use the observations of the LIDAR to update the map using geometric primitives:
                if self.geometry_map is not None:
                    self.geometry_map.update(self.robot, lidar_index, observations, delta_time)

    def draw(self, clock : py.time.Clock):
        """
        Draws the whole scene
        
            :param clock: Clock used to measure the FPS of the simulation
        """

        # Draw the static and dynamic maps if necessary:
        if self.grid_maps is not None:
            if self.display['grid_maps'] == 1:
                self.grid_maps.draw(self, view='static')
            elif self.display['grid_maps'] == 2:
                self.grid_maps.draw(self, view='dynamic')
            elif self.display['grid_maps'] == 3:
                self.grid_maps.draw(self, view='both')

        # Draw the robot:
        self.robot.draw(self, self.display['rays'])
        
        # Draw the map using geometric primitives if necessary:
        if self.geometry_map is not None and self.display['geometry_map']:
            self.geometry_map.draw(self, self.display)

        # Draw the current FPS at which the simulation is running:
        self.draw_text('FPS: %.1f' % clock.get_fps(), 'top_left')

    def on_key_down(self, key):
        """ Should be called when a key button on the keyboard is pressed """

        # If R-key is pressed, toggle the drawing of LIDAR rays:
        if key == py.K_r:
            self.display['rays'] = not self.display['rays']

        # If G-key is pressed, change the draw mode for grid maps:
        elif key == py.K_g:
            # 0: none, 1: static, 2: dynamic, 3: both:
            self.display['grid_maps'] = (self.display['grid_maps'] + 1) % 4

            mode = ['none', 'static', 'dynamic', 'static + dynamic']
            print("Grid maps display:", mode[self.display['grid_maps']])

        # If P-key is pressed, toggle the drawing of the mapping using geometric primitives:
        elif key == py.K_p:
            self.display['geometry_map'] = not self.display['geometry_map']

        # If W-key is pressed, toggle the drawing of the wipe-shape:
        elif key == py.K_w:
            self.display['wipe_shape'] = not self.display['wipe_shape']

        # If M-key is pressed, toggle the drawing of the match-grid (used for geometric maps):
        elif key == py.K_m:
            self.display['match_grid'] = not self.display['match_grid']

        # If S-key is pressed, toggle the drawing of the speed estimate of primitives:
        elif key == py.K_s:
            self.display['speed_estimates'] = not self.display['speed_estimates']

        # If L-key is pressed, toggle the drawing of lines in the geometric maps:
        elif key == py.K_l:
            self.display['lines'] = not self.display['lines']

        # If the C-key is pressed, toggle the drawing of circles in the geometric maps:
        elif key == py.K_c:
            self.display['circles'] = not self.display['circles']

    def draw_text(self, text : str, corner : str, color : tuple = (0, 0, 0)):
        """
        Draws text on the screen
        
            :param text: The text to print
            :param corner: On which corner of the screen to draw the text. One among
                'top_left', 'top_right', 'bottom_left' or 'bottom_right'
            :param color: The color of the text
        """

        surface = self.text_font.render(text, True, color)
        rect = surface.get_rect()

        # Compute the position where the text should be drawn:
        if corner == 'top_left':
            pos = (0, 0)
        elif corner == 'top_right':
            pos = (self.screen.get_width() - rect.width,0)
        elif corner == 'bottom_left':
            pos = (0,self.screen.get_height() - rect.height)
        else:
            pos = (self.screen.get_width() - rect.width, 
                   self.screen.get_height() - rect.height)
            
        # Draw the text:
        self.screen.blit(surface, pos)

    def draw_grid(self, x : float, y : float, cell_size : float, data : np.ndarray):
        """
        Draws a grid in the scene
        
            :param x: x-coordinate of the center of the grid, in meters
            :param y: y-coordinate of the center of the grid, in meters
            :param cell_size: Size of each cell in the grid, in meters
            :param data: 2D ndarray of data, containing grayscales between 0 and 1
        """

        # Create a surface to draw the grid:
        surface = py.Surface(data.shape)

        # Convert the data into grayscales between 0 and 255:
        data = (255 * data).astype(np.uint8, copy=False)

        # Convert to RGB representation by stacking three times the same colors
        # and put the data in the surface:
        py.surfarray.blit_array(surface, np.stack((data, ) * 3, axis=-1))

        # Reshape the surface:
        width = data.shape[0] * cell_size * self.pixels_per_meter
        height = data.shape[1] * cell_size * self.pixels_per_meter
        surface = py.transform.scale(surface, (width, height))

        # Compute the center of the surface:
        rect = surface.get_rect()
        rect.center = self.screen_coordinates(x, y)

        self.screen.blit(surface, rect)

    def draw_arrow(self, start_x : float, start_y : float, width : float, 
                   height : float, angle: float, color : tuple, 
                   body_width_ratio : float = 0.8, body_height_ratio : float = 0.3):
        """
        Draws an arrow in the scene
        
            :param start_x: x-coordinate of the begin point of the arrow in meters
            :param start_y: y-coordinate of the begin point of the arrow in meters
            :param width: Width of the arrow in meters
            :param height: Height of the arrow in meters
            :param angle: Angle of the arrow in radians, counterclockwise
            :param color: Color of the rectangle
            :param body_width_ratio: Width of the 'body' of the arrow (arrow without
                the tip) divided by the width of the arrow
            :param body_height_ratio: Height of the 'body' of the arrow (arrow without
                the tip) divided by the height of the arrow
        """

        # Define the position of the points of a normalized arrow
        # (width = height = 1 and angle = 0):
        body_height_ratio /= 2
        points = np.array([
            (0, body_height_ratio), (body_width_ratio, body_height_ratio),
            (body_width_ratio, 0.5), (1, 0), (body_width_ratio, -0.5),
            (body_width_ratio, -body_height_ratio), (0, -body_height_ratio)])
        
        # Resize the arrow to the correct dimensions:
        points[:,0] *= width * self.pixels_per_meter
        points[:,1] *= height * self.pixels_per_meter

        # Rotate the arrow:
        points = points @ np.array([[np.cos(angle), -np.sin(angle)], 
                                    [np.sin(angle), np.cos(angle)]])
        
        # Put the arrow at the right position:
        start_x, start_y = self.screen_coordinates(start_x, start_y)
        points[:,0] += start_x
        points[:,1] += start_y

        # Draw the arrow:
        py.draw.polygon(self.screen, color, points)

    def draw_rectangle(self, x : float, y : float, width : float, height : float, 
                       angle : float, color : tuple):
        """
        Draws a rectangle in the scene

            :param x: x-coordinate of the center of the rectangle, in meters
            :param y: y-coordinate of the center of the rectangle, in meters
            :param width: Width of the rectangle in meters
            :param height: Height of the rectangle in meters
            :param angle: Angle of the rectangle in radians, counterclockwise
            :param color: Color of the rectangle
        """

        # Convert the position from world to screen space:
        x, y = self.screen_coordinates(x, y)

        # Resize the ractangle:
        width *= self.pixels_per_meter
        height *= self.pixels_per_meter

        # Compute the position of the corners:
        theta = math.atan(height / width)
        radius = math.sqrt(width*width + height*height) / 2

        corners = []
        for offset in [theta, math.pi - theta, math.pi + theta, -theta]:
            corner_x = x + radius * math.cos(offset - angle)
            corner_y = y + radius * math.sin(offset - angle)

            corners.append((corner_x, corner_y))

        py.draw.polygon(self.screen, color, corners)

    def draw_polygon(self, positions : np.ndarray, color : tuple, width : int = 0):
        """
        Draws a polygon in the scene
        
            :param positions: Numpy array of positions of the points of the polygon
                in world space (in meters)
            :param color: Color of tthe polygon
            :param width:
                If width == 0 (default): fills the polygon
                If width > 0: Used for line thickness
                If width < 0: Nothing will be drawn
        """

        # Convert the positions from world space to screen space:
        positions[:,0], positions[:,1] = self.screen_coordinates(positions[:,0], positions[:,1])

        # Draw the polygon:
        py.draw.polygon(self.screen, color, positions, width)

    def draw_line(self, x1 : float, y1 : float, x2 : float, y2 : float, color : tuple):
        """
        Draws a line in the scene
        
            :param x1: x-coordinate of the begin point of the line in meters
            :param y1: y-coordinate of the begin point of the line in meters
            :param x2: x-coordinate of the end point of the line in meters
            :param y2: y-coordinate of the end point of the line in meters
            :param color: color of the line
        """

        x1, y1 = self.screen_coordinates(x1, y1)
        x2, y2 = self.screen_coordinates(x2, y2)

        py.draw.line(self.screen, color, (x1, y1), (x2, y2))

    def draw_circle(self, x : float, y : float, radius : float, color : tuple):
        """
        Draws a circle in the scene
        
            :param x: x-coordinate of the circle in meters
            :param y: y-coordinate of the circle in meters
            :param radius: radius of the circle in meters
            :param color: color of the circle
        """

        center = self.screen_coordinates(x, y)

        py.draw.circle(self.screen, color, center, radius * self.pixels_per_meter)

    def screen_coordinates(self, x : float | np.ndarray, y : float | np.ndarray) -> tuple:
        """ Converts coordinates from world space into screen space """

        screen_x = (x - self.corner_x) * self.pixels_per_meter
        screen_y = (self.corner_y - y) * self.pixels_per_meter

        return (screen_x, screen_y)
    
    def world_coordinates(self, x : float | np.ndarray, y : float | np.ndarray) -> tuple:
        """ Converts coordinates from screen space into world space """

        world_x = self.corner_x + x / self.pixels_per_meter
        world_y = self.corner_y - y / self.pixels_per_meter

        return (world_x, world_y)
    
    def zoom(self, factor : float, screen_center_x : float, screen_center_y : float):
        """ Zoom in the scene, by the given factor
            
            :param factor: Factor by which the camera will zoom in the scene
            :param screen_center_x: Screen position X around which the camera will zoom
            :param screen_center_y: Screen position Y around which the camera will zoom
        """

        ratio = (1 - 1/factor) / self.pixels_per_meter
        self.corner_x += ratio * screen_center_x
        self.corner_y -= ratio * screen_center_y        
        self.pixels_per_meter *= factor

    def move(self, delta_x : float, delta_y : float):
        """ Move the scene by the given amounts in pixels """

        self.corner_x -= delta_x / self.pixels_per_meter
        self.corner_y -= delta_y / self.pixels_per_meter

