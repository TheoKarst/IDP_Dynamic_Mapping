import math
import pygame as py
import numpy as np
from robot import Robot
from dataloader import Dataloader
from mapping.grid_maps.grids_mapping_bresenham import GridsMappingBresenham
from mapping.grid_maps.grids_mapping_rectangle import GridsMappingRectangle
from mapping.geometry_maps.geometry_mapping import GeometryMapping

folder = "C:\\Users\\theok\\IDP\\LidarCaptures\\StaticCapture_0"

class Scene:
    def __init__(self, screen : py.Surface, centerX : float, centerY : float, pixels_per_meter : float):
        self.screen = screen

        # World position of the top left corner of the camera:
        self.corner_x = centerX - screen.get_width() / (2 * pixels_per_meter)
        self.corner_y = centerY + screen.get_height() / (2 * pixels_per_meter)

        self.pixels_per_meter = pixels_per_meter

        # Create a dataloader to load recorded data:
        self.dataloader = Dataloader(folder)

        # Instantiate a robot with the setup defined in the recorded data:
        robot_setup = self.dataloader.load_robot_setup()
        self.robot = Robot(**robot_setup)

        # Instantiate a manager for grid mapping:
        self.grid_maps = GridsMappingRectangle((0,0), 700, 400, 0.1)
        self.grid_maps.setup_using_frames_counts(60, 5, 0.999)

        # Instantiate a manager for mapping using geometric primitives:
        self.geometry_map = GeometryMapping()

        self.draw_maps = True
        self.draw_rays = True

        # For information, print the boundaries of the screen in world space:
        xmin, ymin = self.world_coordinates(0, 0)
        xmax, ymax = self.world_coordinates(screen.get_width(), screen.get_height())
        print(f"Screen setup: [xmin: {xmin}m; xmax: {xmax}m], [ymin: {ymin}m; ymax: {ymax}m]")

    def update(self):
        """ Called each frame, updates the whole scene """

        # Get the current time in seconds (elapsed time since pygame.init() was called):
        current_time = py.time.get_ticks() / 1000

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
                # self.grid_maps.update_maps(lidar_pose, observations)

                # Use the observations of the LIDAR to update the map using geometric primitives:
                self.geometry_map.update(self.robot, lidar_index, observations, current_time)

    def draw(self):
        """ Draws the whole scene """

        # Draw the static and dynamic maps if necessary:
        if self.draw_maps:
            self.grid_maps.draw(self)
            self.geometry_map.draw(self)

        self.robot.draw(self, self.draw_rays)

    def on_key_down(self, key):
        """ Should be called when a key button on the keyboard is pressed """

        # If the M-key is pressed, toggle the drawing of the static and dynamic maps:
        if key == py.K_m:
            self.draw_maps = not self.draw_maps

        if key == py.K_r:
            self.draw_rays = not self.draw_rays

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


    def draw_rectangle(self, x : float, y : float, width : float, height : float, 
                       angle : float, color : tuple):
        """
        Draws a rectangle in the scene

            :param x: x-coordinate of the center of the rectangle, in meters
            :param y: y-coordinate of the center of the rectangle, in meters
            :param width: width of the rectangle in meters
            :param height: height of the rectangle in meters
            :param angle: angle (in radians) of the rectangle, counter-clockwise
            :param color: color of the rectangle
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

    def screen_coordinates(self, x : float, y : float) -> tuple:
        """ Converts coordinates from world space into screen space """

        screen_x = (x - self.corner_x) * self.pixels_per_meter
        screen_y = (self.corner_y - y) * self.pixels_per_meter

        return (screen_x, screen_y)
    
    def world_coordinates(self, x : float, y : float) -> tuple:
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

