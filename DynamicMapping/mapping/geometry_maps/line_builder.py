import numpy as np
from .circle_builder import CircleBuilder

class LineBuilder:

    def __init__(self, initial_point):
        # Last point that was added to the line:
        self.last_point = initial_point

        # Unwrap the tuple representing the point:
        initial_angle, initial_position, initial_covariance = initial_point

        # List of points representing this line:
        self.points = {}
        self.points['angles'] = [initial_angle]
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

    def add_point(self, point):
        """ Adds a point to the line and updates the parameters of the line """

        # Update the last point:
        self.last_point = point

        # Unwrap the tuple representing the point:
        angle, position, covariance = point

        self.points['angles'].append(angle)
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
        n = len(self.points['angles'])

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

    # Return the length of the line, using its endpoints
    def length(self):
        if(not self.up_to_date_endpoints):
            self.update_endpoints()

        return np.sqrt(np.sum((self.begin_point - self.end_point) ** 2))

    # Convert this line into a circle cluster (this line shouldn't be used after that):
    def to_circle(self):
        return CircleBuilder(self.points)

    def build(self, Q):
        # Compute the endpoints and covariance matrix of the line:
        if not self.up_to_date_endpoints: 
            self.update_endpoints()

        covariance = self.compute_covariance()

        # Build the line:
        # line = new DynamicLine(rho, theta, covariance, beginPoint, endPoint, Q);
        line = (self.rho, self.theta, covariance, self.begin_point, self.end_point)

        # Match all the points to the built line:
        # foreach (Point point in points) point.MatchToPrimitive(line);

        return line

    # Compute the endpoints of the line by projecting the first and last points that were added
    # in the line builder, along the line defined by the parameters (rho, theta):
    def update_endpoints(self):
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

    # Compute the covariance matrix of the parameters (rho, theta) of the line:
    def compute_covariance(self):
        # TODO: Implement this !
        return None

    # private Matrix<double> ComputeCovariance() {
    #     // Step 1: Compute Cv (covariance matrix on m,q or s,t):
    #     Matrix<double> Cv = M.Dense(2, 2, 0);    // 2*2 Zero Matrix

    #     int n = points.Count;
    #     float N1 = Rxx * n - Rx * Rx;
    #     float N2 = Ryy * n - Ry * Ry;
    #     float T = Rxy * n - Rx * Ry;

    #     Matrix<double> Hl;
    #     if (N1 >= N2) {     // We use (m,q) representation
    #         float m = T / N1;
    #         float q = (Ry - m * Rx) / n;

    #         for (int i = 0; i < points.Count; i++) {
    #             Matrix<double> J = JacobianMQi(m, N1, T, i);
    #             Matrix<double> Cp = points[i].Cp;
    #             Cv += J * Cp.TransposeAndMultiply(J);
    #         }

    #         // Compute Hl, Jacobian of (rho, theta) with respect to (m, q):
    #         float tmp = 1 + m * m;
    #         float drho_dm = -m * Mathf.Abs(q) / Mathf.Pow(tmp, 1.5f);
    #         float drho_dq = Mathf.Sign(q) / Mathf.Sqrt(tmp);
    #         float dtheta_dm = 1 / tmp;
    #         float dtheta_dq = 0;

    #         Hl = M.DenseOfArray(new double[,] {
    #             { drho_dm,      drho_dq },
    #             { dtheta_dm,    dtheta_dq } });
    #     }
    #     else {              // We use (s,t) representation
    #         float s = T / N2;
    #         float t = (Rx - s * Ry) / n;

    #         for (int i = 0; i < points.Count; i++) {
    #             Matrix<double> J = JacobianSTi(s, N2, T, i);
    #             Matrix<double> Cp = points[i].Cp;
    #             Cv += J * Cp.TransposeAndMultiply(J);
    #         }

    #         // Compute Hl, Jacobian of (rho, theta) with respect to (s, t):
    #         float tmp = 1 + s * s;
    #         float drho_ds = -s * Mathf.Abs(t) / Mathf.Pow(tmp, 1.5f);
    #         float drho_dt = Mathf.Sign(t) / Mathf.Sqrt(tmp);
    #         float dtheta_ds = -1 / tmp;
    #         float dtheta_dt = 0;

    #         Hl = M.DenseOfArray(new double[,] {
    #             { drho_ds,      drho_dt },
    #             { dtheta_ds,    dtheta_dt } });
    #     }

    #     // Setp 2: Use the previously computed Cv and Hl to compute the covariance
    #     // matrix of the parameters (rho, theta) of the line:
    #     return Hl * Cv.TransposeAndMultiply(Hl);
    # }

    # private Matrix<double> JacobianMQi(float m, float N1, float T, int i) {
    #     int n = points.Count;
    #     float xpi = points[i].position.x, ypi = points[i].position.y;

    #     float dm_dx = (N1 * (n * ypi - Ry) - 2 * T * (n * xpi - Rx)) / (N1 * N1);
    #     float dm_dy = (n * xpi - Rx) / N1;
    #     float dq_dx = -(Rx * dm_dx + m) / n;
    #     float dq_dy = (1 - Rx * dm_dy) / n;

    #     return M.DenseOfArray(new double[,] { 
    #         { dm_dx, dm_dy }, 
    #         { dq_dx, dq_dy } });
    # }

    # private Matrix<double> JacobianSTi(float s, float N2, float T, int i) {
    #     int n = points.Count;
    #     float xpi = points[i].position.x, ypi = points[i].position.y;

    #     float ds_dx = (n * ypi - Ry) / N2;
    #     float ds_dy = (N2 * (n * xpi - Rx) - 2 * T * (n * ypi - Ry)) / (N2 * N2);
    #     float dt_dx = (1 - Ry * ds_dx) / n;
    #     float dt_dy = -(Ry * ds_dy + s) / n;

    #     return M.DenseOfArray(new double[,] {
    #         { ds_dx, ds_dy },
    #         { dt_dx, dt_dy } });
    # }

    def distance_from(self, position):
        """ Returns the distance between the line and the given position """

        return abs(position[0] * np.cos(self.theta) + position[1] * np.sin(self.theta) - self.rho)

    def points_count(self):
        return len(self.points['angles'])