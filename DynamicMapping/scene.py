import math
import pygame as py
from robot import Robot
from dataloader import Dataloader

folder = "C:\\Users\\theok\\IDP\\LidarCaptures\\StaticCapture_0"

class Scene:
    def __init__(self, screen : py.Surface, centerX, centerY, pixels_per_meter):
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

        # For information, print the boundaries of the screen in world space:
        xmin, ymin = self.world_coordinates(0, 0)
        xmax, ymax = self.world_coordinates(screen.get_width(), screen.get_height())
        print(f"Screen setup: [xmin: {xmin}m; xmax: {xmax}m], [ymin: {ymin}m; ymax: {ymax}m]")

    def update(self):
        """ Called each frame, updates the whole scene """

        data_frame = self.dataloader.load_current_frame(py.time.get_ticks() / 1000)

        if data_frame is not None:
            self.robot.update(**data_frame)

    def draw(self):
        """ Draws the whole scene """

        self.robot.draw(self)

    def draw_rectangle(self, x, y, width, height, angle, color):
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

    def draw_line(self, x1, y1, x2, y2, color):
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

    def draw_circle(self, x, y, radius, color):
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

