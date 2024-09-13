import numpy as np

# from scene.scene import Scene

class Circle:

    def __init__(self, center : np.ndarray, R : float):
        """
        Instantiates a circle to represent a cluster of points
        
            :param center: Center of the circle in meters
            :param R: Radius of the circle in meters
        """
        
        # Center and radius of the circle:
        self.center = center
        
        # Derivative of center_x and center_y with respect to time:
        self.speed = np.array([0.0, 0.0])

        # Radius of the circle:
        self.R = R

        # If the circle is consistent with the current observations:
        self.is_valid = True

        # Color of the circle (for drawing):
        self.circle_color = (255, 0, 0)

    def draw(self, scene : 'Scene', draw_speed_estimate : bool = False):
        """ Draws the circle in the scene """

        scene.draw_circle(self.center[0], self.center[1], self.R, self.circle_color)

        if draw_speed_estimate:
            angle = np.arctan2(self.speed[1], self.speed[0])
            length = np.sqrt(np.sum(self.speed ** 2))
            scene.draw_arrow(self.center[0], self.center[1], self.R, length, angle, (255, 0, 0))

    def predict_state(self, elapsed_time : float, friction : float):
        """
        From the previous circle estimate and the elapsed time since the last update,
        predict where the circle should be now.

            :param elapsed_time: Elapsed time since the last update
            :param friction: Friction applied to the speed of the circle (between 0 and 1)
        """

        self.center += elapsed_time * self.speed
        self.speed *= 1 - friction

    def update_circle_using_match(self, other : 'Circle'):
        """
        Supposing that this circle belongs to the current model of the environment, use the
        given circle (supposed to be an observed circle, matched with this one) to update the
        center and radius of this circle
        """

        new_center = (self.center + other.center) / 2
        self.R = (self.R + other.R) / 2

        # Use the new_xc and new_yc to update the circle speed estimate, using a simple
        # exponential low pass filter (TODO: Use Kalman Filter instead):
        m = 0.95
        self.speed = m * self.speed + (1 - m) * (new_center - self.center)

        # Update the center of the circle:
        self.center = new_center

    def distance_from(self, other : 'Circle'):
        """
        Computes the Euclidean distance between the center of this circle and
        the center of the given circle
        """

        return np.sqrt(np.sum((other.center - self.center) ** 2))
    
    def is_far_from_lines(self, lines, min_distance_to_lines : float):
        """
        Returns if the circle is far enough from the given lines
        
            :param lines: Iterable list of lines
            :param min_distance_to_lines: Minimum allowed orthogonal distance 
                between a line and the center of the circle
        """

        for line in lines:
            if line.distance_of(self.center) < min_distance_to_lines:
                return False
            
        return True