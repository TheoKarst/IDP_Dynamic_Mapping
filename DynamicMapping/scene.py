import pygame as py
from dataloader import Dataloader

folder = "C:\\Users\\theok\\IDP\\LidarCaptures\\StaticCapture_0"

class Scene:
    def __init__(self, screen : py.Surface, centerX, centerY, pixels_per_meter):
        self.screen = screen

        # World position of the top left corner of the camera:
        self.corner_x = centerX - screen.get_width() / (2 * pixels_per_meter)
        self.corner_y = centerY + screen.get_height() / (2 * pixels_per_meter)

        self.pixels_per_meter = pixels_per_meter

        self.dataloader = Dataloader(folder)

        self.robot = self.dataloader.load_robot_setup()

        xmin, ymin = self.world_coordinates(0, 0)
        xmax, ymax = self.world_coordinates(screen.get_width(), screen.get_height())
        print(f"Screen setup: [{xmin}; {xmax}], [{ymin}; {ymax}]")

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

        screen_x = (x - self.corner_x) * self.pixels_per_meter
        screen_y = (self.corner_y - y) * self.pixels_per_meter

        return (screen_x, screen_y)
    
    def world_coordinates(self, x : float, y : float) -> tuple:
        """ Convert coordinates from screen space into world space """

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
        self.corner_x -= delta_x / self.pixels_per_meter
        self.corner_y -= delta_y / self.pixels_per_meter

