import numpy as np

from geometry_maps.primitives.circle_builder import CircleBuilder
from geometry_maps.primitives.dynamic_line import DynamicLine

class LineBuilder:

    def __init__(self, initial_position : np.ndarray, initial_covariance : np.ndarray):
        """
        Instantiates a class to build lines from points
        
            :param initial_position: Position of the first point to add to the line
            :param initial_covariance: 2x2 covariance matrix of the initial position
        """

        # List of points representing this line:
        self.points = {}
        self.points['positions'] = [initial_position]
        self.points['covariances'] = [initial_covariance]

        # Regression parameters of the line:
        self.Rx = initial_position[0]
        self.Ry = initial_position[1]
        self.Rxx = initial_position[0] * initial_position[0]
        self.Ryy = initial_position[1] * initial_position[1]
        self.Rxy = initial_position[0] * initial_position[1]

        # Parameters defining the line:
        self.rho = 0
        self.theta = 0

        # Current endpoints of the line and a flag saying if they are up to date:
        self.begin_point = None
        self.end_point = None
        self.up_to_date_endpoints = False

    def add_point(self, position : np.ndarray, covariance : np.ndarray):
        """
        Adds a point to the line and updates the parameters of the line
        
            :param position: 2D world position of the point
            :param covariance: 2x2 covariance matrix of the position
        """

        self.points['positions'].append(position)
        self.points['covariances'].append(covariance)

        # Now the endpoints are not up to date anymore:
        self.up_to_date_endpoints = False

        self.Rx += position[0]
        self.Ry += position[1]
        self.Rxx += position[0] * position[0]
        self.Ryy += position[1] * position[1]
        self.Rxy += position[0] * position[1]

        # Get the number of points in the line:
        n = len(self.points['positions'])

        N1 = self.Rxx * n - self.Rx * self.Rx
        N2 = self.Ryy * n - self.Ry * self.Ry
        T = self.Rxy * n - self.Rx * self.Ry

        # N1 and N2 represent the width of the cloud of the regression points along the 
        # X- and Y-axis. If N1 is larger than N2, the cloud of points lies more horizontally 
        # than vertically which makes regression of y to x (y = mx + q) more favourable.
        # Otherwise, the regression of x to y is selected (x = sy + t):
        if N1 >= N2:
            m = T / N1
            q = (self.Ry - m * self.Rx) / n

            self.rho = abs(q / np.sqrt(m * m + 1))
            self.theta = np.arctan2(q, -q * m)
        else:
            s = T / N2
            t = (self.Rx - s * self.Ry) / n

            self.rho = abs(t / np.sqrt(s * s + 1))
            self.theta = np.arctan2(-t * s, t)

    def length(self):
        """ Returns the length of the line in meters, using its endpoints """

        if(not self.up_to_date_endpoints):
            self.update_endpoints()

        return np.sqrt(np.sum((self.begin_point - self.end_point) ** 2))

    def to_circle(self):
        """ Convert this line into a circle cluster (this line shouldn't be used after that) """

        return CircleBuilder(self.points)

    def build(self):
        """ Returns a dynamic line instance, built from the points added to this line builder """

        # Compute the endpoints and covariance matrix of the line:
        if not self.up_to_date_endpoints: 
            self.update_endpoints()

        covariance = self.compute_covariance()

        # Build the line:
        return DynamicLine(self.rho, self.theta, covariance, self.begin_point, self.end_point)

    def update_endpoints(self):
        """
        Computes the endpoints of the line by projecting the first and last points that were added
        in the line builder, along the line defined by the parameters (rho, theta)
        """

        # Intermediate computations:
        costheta = np.cos(self.theta)
        sintheta = np.sin(self.theta)

        # Unit vector along the line:
        u = np.array([-sintheta, costheta])        

        # Compute the projection of the first point and last point of the line:
        p_first = np.sum(u * self.points['positions'][0])
        p_last = np.sum(u * self.points['positions'][-1])

        # "Center" of the infinite line:
        center = self.rho * np.array([costheta, sintheta])
        
        self.begin_point = center + p_first * u
        self.end_point = center + p_last * u
        self.up_to_date_endpoints = True

    def compute_covariance(self):
        """ Compute the covariance matrix of the parameters (rho, theta) of the line """

        # We first convert the array of points into Numpy arrays:
        positions = np.array(self.points['positions'])
        covariances = np.array(self.points['covariances'])

        n = len(positions)
        N1 = self.Rxx * n - self.Rx * self.Rx
        N2 = self.Ryy * n - self.Ry * self.Ry
        T = self.Rxy * n - self.Rx * self.Ry

        # If we are using (m, q) representation:
        if N1 >= N2:
            m = T / N1
            q = (self.Ry - m * self.Rx) / n

            # First compute the covariance matrix of m, q:
            Cv = self.compute_mq_covariance(positions, covariances, m, N1, T)

            # Then compute the Jacobian of (rho, theta) with respect to (m, q):
            drho_dm = -m * abs(q) / ((1 + m*m) ** (3/2))
            drho_dq = np.sign(q) / np.sqrt(m*m + 1)
            dtheta_dm = 1 / (1 + m*m)
            dtheta_dq = 0

            Hl = np.array([[drho_dm,    drho_dq  ], 
                           [dtheta_dm,  dtheta_dq]])

        # Else, if we are using (s, t) representation:
        else:
            s = T / N2
            t = (self.Rx - s * self.Ry) / n

            # First compute the covariance matrix of s, t:
            Cv = self.compute_st_covariance(positions, covariances, s, N2, T)

            # Then compute the Jacobian of (rho, theta) with respect to (s, t):
            drho_ds = -s * np.sign(t) / ((1 + s*s) ** (3/2))
            drho_dt = np.sign(t) / np.sqrt(1 + s*s)
            dtheta_ds = -1 / (1 + s*s)
            dtheta_dt = 0

            Hl = np.array([[drho_ds,    drho_dt  ],
                           [dtheta_ds,  dtheta_dt]])
            
        # The covariance matrix of (rho, theta) can finally be computed using:
        return Hl @ Cv @ Hl.T
    
    def compute_mq_covariance(self, positions : np.ndarray, covariances : np.ndarray, 
                              m : float, N1 : float, T : float):
        """ Computes the covariance matrix of the (m, q) representation of the line """

        n = len(positions)

        # Unwrap the columns of the positions:
        xi, yi = positions[:,0], positions[:,1]

        # We have: (m, q) = f((x1, y1), ..., (xn, yn)).
        # We need to compute the Jacobian of f with respect to each point (xi, yi).
        # To do this, all the Jacobians are stacked together:
        J = np.zeros((n, 2, 2))

        # Derivative of m with respect to xi:
        J[:,0,0] = ((n * yi - self.Ry) * N1 - 2 * T * (n*xi- self.Rx)) / (N1*N1)

        # Derivative of m with respect to yi:
        J[:,0,1] = (n * xi - self.Rx) / N1

        # Derivative of q with respect to xi:
        J[:,1,0] = -(self.Rx * J[:,0,0] + m) / n

        # Derivative of q with respect to yi:
        J[:,1,1] = (1 - self.Rx * J[:,0,1]) / n

        # We can compute the covariance matrix of m, q using:
        return np.sum(J @ covariances @ np.transpose(J, axes=(0,2,1)), axis=0)
    
    def compute_st_covariance(self, positions : np.ndarray, covariances : np.ndarray,
                              s : float, N2 : float, T : float):
        """ Computes the covariance matrix of the (s, t) representation of the line """

        n = len(positions)

        # Unwrap the columns of the positions:
        xi, yi = positions[:,0], positions[:,1]

        # We have: (s, t) = f((x1, y1), ..., (xn, yn)).
        # We need to compute the Jacobian of f with respect to each point (xi, yi).
        # To do this, all the Jacobians are stacked together:
        J = np.zeros((n, 2, 2))

        # Derivative of s with respect to xi:
        J[:,0,0] = (n * yi - self.Ry) / N2

        # Derivative of s with respect to yi:
        J[:,0,1] = ((n * xi - self.Rx) * N2 - 2 * T * (n * yi - self.Ry)) / (N2 * N2)

        # Derivative of t with respect to xi:
        J[:,1,0] = (1 - self.Ry * J[:,0,0]) / n

        # Derivative of t with respect to yi:
        J[:,1,1] = -(self.Ry * J[:,0,1] + s) / n

        # We can compute the covariance matrix of s, t using:
        return np.sum(J @ covariances @ np.transpose(J, axes=(0,2,1)), axis=0)
    
    def distance_from(self, position):
        """ Returns the distance between the line and the given position """

        return abs(position[0] * np.cos(self.theta) + position[1] * np.sin(self.theta) - self.rho)

    def points_count(self):
        """ Returns the number of points currently in the line """

        return len(self.points['positions'])