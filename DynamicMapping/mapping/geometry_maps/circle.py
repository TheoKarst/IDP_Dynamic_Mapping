import numpy as np

class Circle:

    def __init__(self, xc, yc, R):

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


    def draw(self, scene : 'Scene'):
        scene.draw_circle(self.xc, self.yc, self.R, self.circle_color)

    # Supposing that this circle belongs to the current model of the environment, use the
    # given circle (that is supposed to be matched with this one) to update the center and
    # radius of this circle:
    def update_circle_using_matching(self, other):
        new_xc = (self.xc + other.xc) / 2
        new_yc = (self.yc + other.yc) / 2
        self.R = (self.R + other.R) / 2

        # Use the new_xc and new_yc to update the circle speed estimate, using a simple
        # exponential low pass filter (TODO: Correct this):
        m = 0.95
        other.d_xc = self.d_xc = m * self.d_xc + (1 - m) * (new_xc - xc)
        other.d_yc = self.d_yc = m * self.d_yc + (1 - m) * (new_yc - yc)

        # Update xc and yc:
        xc = new_xc
        yc = new_yc

    # Compute the Euclidean distance between the two circles centers:
    def distance_from(self, other : 'Circle'):
        dX = other.xc - self.xc
        dY = other.yc - self.yc

        return np.Sqrt(dX * dX + dY * dY)

    # All the points belonging to a circle have the same speed, which is the speed
    # of the center of the circle:
    def velocity_of_point(self, x, y):
        return self.d_xc, self.d_yc