import numpy as np

class CircleBuilder:

    def __init__(self, points : dict | None = None):
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

    def add_point(self, position, covariance):        
        self.points['positions'].append(position)
        self.points['covariances'].append(covariance)
        
        self.Rxy += position

        self.center = self.Rxy / len(self.points['positions'])

    def build(self):
        # Compute the radius of the circle:
        R = self.compute_circle_radius()

        # Build the circle:
        # circle = Circle(center.x, center.y, R);
        circle = (self.center, R)

        # Match all the points to the built circle:
        # foreach (Point point in points) point.MatchToPrimitive(circle);

        return circle

    def compute_circle_radius(self):
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
        return len(self.points['positions'])