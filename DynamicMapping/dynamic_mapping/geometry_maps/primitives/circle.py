import numpy as np

# from scene.scene import Scene

class Circle:

    def __init__(self, xc, yc, R):
        """
        Instantiates a circle to represent a cluster of points
        
            :param xc: x-position of the center of the circle in meters
            :param yc: y-position of the center of the circle in meters
            :param R: Radius of the circle in meters
        """
        
        # Center and radius of the circle:
        self.xc = xc
        self.yc = yc
        self.R = R

        # Derivative of xc and yc (used to estimate the speed of the circle):
        self.d_xc = 0
        self.d_yc = 0

        # If the circle is consistent with the current observations:
        self.is_valid = True

        # Color of the circle (for drawing):
        self.circle_color = (255, 0, 0)

    def draw(self, scene : 'Scene', draw_speed_estimate : bool = False):
        """ Draws the circle in the scene """

        scene.draw_circle(self.xc, self.yc, self.R, self.circle_color)

        if draw_speed_estimate:
            angle = np.arctan2(self.d_yc, self.d_xc)
            length = np.sqrt(self.d_xc ** 2 + self.d_yc ** 2)
            scene.draw_arrow(self.xc, self.yc, self.R, length, angle, (255, 0, 0))

    def update_circle_using_matching(self, other : 'Circle'):
        """
        Supposing that this circle belongs to the current model of the environment, use the
        given circle (supposed to be an observed circle, matched with this one) to update the
        center and radius of this circle
        """

        new_xc = (self.xc + other.xc) / 2
        new_yc = (self.yc + other.yc) / 2
        self.R = (self.R + other.R) / 2

        # Use the new_xc and new_yc to update the circle speed estimate, using a simple
        # exponential low pass filter (TODO: Use Kalman Filter instead):
        m = 0.95
        self.d_xc = m * self.d_xc + (1 - m) * (new_xc - xc)
        self.d_yc = m * self.d_yc + (1 - m) * (new_yc - yc)

        # Update xc and yc:
        xc = new_xc
        yc = new_yc

    def distance_from(self, other : 'Circle'):
        """
        Computes the Euclidean distance between the center of this circle and
        the center of the given circle
        """

        dX = other.xc - self.xc
        dY = other.yc - self.yc

        return np.Sqrt(dX * dX + dY * dY)