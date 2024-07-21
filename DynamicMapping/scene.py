import pygame as py
from dataloader import Dataloader

folder = "C:\\Users\\theok\\IDP\\LidarCaptures\\StaticCapture_0"

class Scene:
    def __init__(self, screen : py.Surface, xmin, xmax, ymin, ymax):
        self.screen = screen
        self.xmin = min(xmin, xmax)
        self.ymin = min(ymin, ymax)

        self.scene_width = abs(xmax - xmin)
        self.scene_height = abs(ymax - ymin)

        self.dataloader = Dataloader(folder)

        self.robot = self.dataloader.load_robot_setup()

    def update(self):
        self.dataloader.update(self.robot, py.time.get_ticks() / 1000)

    def draw(self):
        self.robot.draw(self)

    def draw_surface(self, surface : py.Surface, x : float, y : float, angle : float = 0):
        """ Convert the given world space pose of a surface into screen space coordinates, 
            and draw the surface on the screen
             
            :param x: World position of the surface along the x-axis
            :param y: World position of the surface along the y-axis
            :param angle: World angle of the surface (in degrees)
        """

        if angle != 0:
            surface = py.transform.rotate(surface, angle)

        rect = surface.get_rect()
        rect.center = self.screen_coordinates(x, y)

        self.screen.blit(surface, rect)

    def draw_line(self, x1, y1, x2, y2, color):
        x1, y1 = self.screen_coordinates(x1, y1)
        x2, y2 = self.screen_coordinates(x2, y2)

        py.draw.line(self.screen, color, (x1, y1), (x2, y2))

    def draw_circle(self, x, y, radius, color):
        center = self.screen_coordinates(x, y)

        py.draw.circle(self.screen, color, center, radius)

    def screen_coordinates(self, x : float, y : float) -> tuple:
        """ Convert coordinates from world space into screen space """

        screen_x = (x - self.xmin) * self.screen.get_width() / self.scene_width
        screen_y = (y - self.ymin) * self.screen.get_height() / self.scene_height

        return (screen_x, screen_y)
        