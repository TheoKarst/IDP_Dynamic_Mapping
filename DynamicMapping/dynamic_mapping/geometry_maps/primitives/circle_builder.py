import numpy as np

class CircleBuilder:

    def __init__(self, points : dict | None = None):
        """
        Instantiates a class to build circles from points
        
            :param points: The current points we want to add in the circle
        """

        # Regression parameters of the circle [Rx, Ry]:
        self.Rxy = np.zeros(2)

        # Points representing this circle:
        if points is None:
            self.points = {}
            self.points['positions'] = []
            self.points['covariances'] = []
        
        else:
            self.points = points

            for position in points['positions']:
                self.Rxy += position

        self.center = self.Rxy / len(self.points['positions'])

    def add_point(self, position : np.ndarray, covariance : np.ndarray):    
        """ Adds a point with the given position (x, y) and covariance to the circle """

        self.points['positions'].append(position)
        self.points['covariances'].append(covariance)
        
        self.Rxy += position

        self.center = self.Rxy / len(self.points['positions'])

    def build(self):
        """ Returns a circle instance, built from the points added to this circle builder """

        # Compute the radius of the circle:
        R = self.compute_circle_radius()

        # Build the circle:
        circle = (self.center, R)

        return circle

    def compute_circle_radius(self):
        """ Computes the radius of the circle from the position of the points """

        n = len(self.points['positions'])

        if (n == 1):
            return 0

        sigma = np.zeros(2)
        for position in self.points['positions']:
            sigma += (position - self.center) ** 2
        
        # The division by 'n-1' instead of 'n' is known as Bessel's correction,
        # which corrects the bias in the estimation of the points variance:
        return np.sqrt(np.sum(sigma / (n - 1)))

    def distance_from(self, point):
        """ Returns the distance between the circle center and the given point """
        
        return np.sqrt(np.sum((point[1] - self.center) ** 2))

    def points_count(self):
        """ Returns the number of points currently inside the circle """

        return len(self.points['positions'])