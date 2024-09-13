import numpy as np

import geometry_maps.utils.angles as utils
from geometry_maps.utils.bool_axis import BoolAxis

class DynamicLine:

    def __init__(self, rho, theta, covariance, begin_point, end_point):
        self.state = {}
        self.state['rho'] = rho         # Range of the line
        self.state['theta'] = theta     # Angle of the line (in radians)
        self.state['der_rho'] = 0       # Derivative of rho with respect to time
        self.state['der_theta'] = 0     # Derivative of theta with respect to time

        self.covariance = np.zeros((4,4))
        self.covariance[:2,:2] = covariance

        # When building a new line, we cannot compute the covariance of der_rho and der_theta
        # because there is no way to estimate the speed of a line with one observation.
        # Even if the covariance matrix will be correctly updated later by the Kalman Filter,
        # the initialisation is still a problem...
        # self.covariance[2,2] = 0  # Initial error estimate on der_rho
        # self.covariance[3,3] = 0  # Initial error estimate on der_theta
        
        self.begin_point = begin_point
        self.end_point = end_point

        # If the line is considered to be static or dynamic. At the beginning,
        # lines are considered to be dynamic but they can become static if they
        # are not moving a lot:
        self.is_static = False

        # Match count the last time the line was considered dynamic:
        self.last_match_dynamic = 0

        # Number of times this line has been matched with an observation:
        self.match_count = 0

        # Use a BoolAxis to represent which parts of the line are "valid":
        self.line_validity = BoolAxis(False)

        # Save the state of the line when we called ResetLineValidity() for the last time:
        self.line_validity_begin = None     # begin_point of the line
        self.line_validity_delta = None     # endPoint - beginPoint
        self.line_validity_u = None         # line_validity_delta / line_validity_delta.magSq
        
        # Save the minimum and maximum projection values of all the lines matched to this one,
        # along line_validity_u:
        self.line_validity_min_p = 0
        self.line_validity_max_p = 0

        # Color of the line (for drawing):
        self.color = (255, 0, 0)

        self.check_state()
    
    def draw(self, scene : 'Scene'):
        color = (0, 0, 0) if self.is_static else self.color

        scene.draw_line(self.begin_point[0], self.begin_point[1], 
                        self.end_point[0], self.end_point[1], color)

    def predict_state(self, elapsed_time : float, process_noise : np.ndarray, friction : float):
        """
        From the previous line estimate and the elapsed time since the last update,
        predict where the line should be now.

            :param elapsed_time: Elapsed time since the last update
            :param process_noise: Process noise matrix for the dynamic lines (4x4 diagonal matrix)
            :param friction: Friction applied to the speed of the line (between 0 and 1)
        """

        # If the line is static, there is nothing to do:
        if (self.is_static):
            return

        # The factor by which the velocity is multiplied with at each frame:
        a = 1 - friction

        # State prediction: we use Euler integration method to compute the
        # new position of the line from the speed of the line and we add 
        # friction to the speed:
        self.state['rho'] += elapsed_time * self.state['der_rho']
        self.state['theta'] += elapsed_time * self.state['der_theta']
        self.state['der_rho'] *= a
        self.state['der_theta'] *= a

        # We then compute the prediction for the line covariance matrix:
        # covariance = F.dot(covariance.dot(F.T)) + Q
        F = np.array([[1, 0, elapsed_time,      0      ],
                      [0, 1,      0      , elapsed_time],
                      [0, 0,      a      ,      0      ],
                      [0, 0,      0      ,      a      ]])

        self.covariance = F @ self.covariance @ F.T + process_noise

        # We also need to update the endpoints of the line. To do so, we
        # simply project them on the new line:
        self.project_endpoints()

        # We finally make sure that the new state is well defined:
        self.check_state()

    def project_endpoints(self):
        """ 
        Replace the endpoints by their orthogonal projection along the line defined
        by the parameters (rho, theta)
        """

        costheta, sintheta = np.cos(self.state['theta']), np.sin(self.state['theta'])

        # Unit vector along the line:
        u = np.array([-sintheta, costheta])

        # "Center" of the infinite line:
        center = self.state['rho'] * np.array([costheta, sintheta])

        self.begin_point = center + np.sum(self.begin_point * u) * u
        self.end_point = center + np.sum(self.end_point * u) * u

    def update_line_using_match(self, observation : 'DynamicLine', 
        validity_margin : float, static_max_der_rho : float, static_max_der_theta : float, 
        min_matches_to_consider_static : float):

        """
        Supposing that this line belongs to the current model of the environment,
        use the given line (that is supposed to be an observed line, matched with 
        this one) to update the position estimate, covariance matrix and endpoints 
        of this line.
        """

        # First, update the state of this line from the observation, using
        # Kalman Filter:
        self.update_state(observation.state['rho'], observation.state['theta'], 
            observation.covariance[:2,:2])

        # Check if the line should now be treated as dynamic or static:
        self.check_dynamic_status(static_max_der_rho, static_max_der_theta, 
                                  min_matches_to_consider_static)
        
        # Then, since the observation is by definition valid, use it to
        # update which sections of this line are valid:
        # UpdateLineValidity(observation.begin_point, observation.end_point, validity_margin);

        # Finally, increase the match count:
        self.match_count += 1
        
    def update_state(self, observation_rho, observation_theta, observation_covariance):
        """ Use the given observation to update the line state 
            :param observation_rho: Range of the observed line
            :param observation_theta: Angle of the observed line
            :param observation_covariance: 2x2 covariance matrix of the observation
        """

        # Compute the innovation vector:
        innovation = np.array([
            observation_rho - self.state['rho'],
            utils.line_substract_angles(observation_theta, self.state['theta'])
        ])

        # Compute the innovation covariance:
        # S = H.dot(covariance.dot(H.T)) + R
        #
        # Where:
        # H = [[1 0 0 0],      R = observationCovariance
        #      [0 1 0 0]]

        # Looking at H, we can see that multiplying with H only 'selects' some 
        # rows and columns of a matrix so computations can be simplified:
        S = self.covariance[:2,:2] + observation_covariance
        
        # Compute the optimal Kalman Gain:
        # K = covariance.dot(H.T).dot(inverse(S))
        K = self.covariance[:,:2] @ np.linalg.inv(S)

        # Compute the updated state estimate:
        # state = state - K.dot(innovation)
        delta_state = K @ innovation
        self.state['rho'] += delta_state[0]
        self.state['theta'] += delta_state[1]
        self.state['der_rho'] += delta_state[2]
        self.state['der_theta'] += delta_state[3]

        # Compute the updated state covariance:
        # covariance = covariance - K * H * covariance:
        self.covariance -= K @ self.covariance[:2,:]

        # Make sure that the new state is well defined:
        self.check_state()

    def check_dynamic_status(self, static_max_der_rho, static_max_der_theta, min_matches_to_consider_static):
        # We check if the line is currently moving:
        is_moving = abs(self.state['der_rho']) > static_max_der_rho \
                or abs(self.state['der_theta']) > static_max_der_theta

        # If we are currently moving, save the current match count, and make sure the line
        # is considered as dynamic:
        if is_moving:
            self.last_match_dynamic = self.match_count
            self.is_static = False

        # Else, if the line is static, make sure that the speed is zero:
        elif self.is_static:
            self.state['der_rho'] = 0
            self.state['der_theta'] = 0

        # Otherwise, if the line is not moving for long enough, we can consider this line to be static:
        elif self.match_count - self.last_match_dynamic >= min_matches_to_consider_static:
            self.is_static = True
            self.state['der_rho'] = 0
            self.state['der_theta'] = 0

    def is_match_candidate(self, model_line : 'DynamicLine', max_delta_angle : float, 
                           max_orthogonal_distance : float, max_parallel_distance):
        """
        Returns if the given line (supposed to be part of the model) is a good candidate
        to be matched with this line (supposed to be an observed line)

            :param model_line: Line in the model we are evaluating
            :param max_delta_angle: Maximum angle in radians between two lines
            :param max_orthogonal_distance: Maximum orthogonal distance between the endpoints of this line
                and the model_line
            :param max_parallel_distance: Maximum distance between the endpoints of this line and the
                model_line, along the model_line
            :returns: If the given model_line fulfils the previous conditions
        """

        # If the angular difference between both lines is too big, the lines cannot match:
        if utils.line_delta_angle(self.state['theta'], model_line.state['theta']) > max_delta_angle:
            return False

        # If the orthogonal distance of the endpoints of this line
        # from the model line are too big, the lines cannot match:
        if model_line.distance_of(self.begin_point) > max_orthogonal_distance:
            return False

        if model_line.distance_of(self.end_point) > max_orthogonal_distance:
            return False

        # Finally, we check the minimum distance between the lines endpoints, along the model line:

        # Compute the unit vector along the model line:
        u = np.array([-np.sin(model_line.state['theta']), np.cos(model_line.state['theta'])])

        # Project all the endpoints along the model line:
        p1 = np.sum(self.begin_point * u)
        p2 = np.sum(self.end_point * u)
        p3 = np.sum(model_line.begin_point * u)
        p4 = np.sum(model_line.end_point * u)

        # Make sure p1 <= p2 and p3 <= p4:
        if p1 > p2:
            (p1, p2) = (p2, p1)

        if p3 > p4:
            (p3, p4) = (p4, p3)

        # The model_line is a good match candidate only if the distance between
        # the intervals [p1, p2] and [p3, p4] is less than the given distance:
        return max(p3 - p2, p1 - p4) < max_parallel_distance

    def norm_distance_from_model(self, model_line : 'DynamicLine'):
        """
        Compute the Mahalanobis distance between this line (supposed to be an observed line)
        and the given one (supposed to be part of the model)
        """

        # Since the speed of the observed lines is unknown (we cannot estimate the speed of
        # a line from a single frame), we only use rho and theta to compute the Norm Distance,
        # and ignore the speed of the line in the model:

        # Extract the covariance matrices of (rho, theta) for both lines:
        Cl = self.covariance[:2,:2]
        Cm = model_line.covariance[:2,:2]
        
        # Compute X = Xl - Xm, and make sure that the difference between the
        # angle of two lines stays in the range ]-pi/2, pi/2]:
        X = np.array([
            self.state['rho'] - model_line.state['rho'],
            utils.line_substract_angles(self.state['theta'], model_line.state['theta'])
        ])

        return X @ np.linalg.inv(Cl + Cm) @ X

    def reset_line_validity(self, start_valid : bool, changes : list[float]):
        """
        Sets which parts of the line are valid or invalid, according to the current observations.
        
            :param start_valid: If the begin_point of this line should be marked as valid
            :param changes: Sorted list of floats, between 0 (beginPoint, excluded) and 1 
            (endPoint, excluded) representing when we change from valid to invalid (or 
            the opposite)
        """

        # Save the current position of the line:
        self.line_validity_begin = self.begin_point
        self.line_validity_delta = self.end_point - self.begin_point
        self.line_validity_u = self.line_validity_delta / np.sum(self.line_validity_delta ** 2)

        # Reset the minimum and maximum projection along the lineValidityU:
        self.line_validity_min_p = 0
        self.line_validity_max_p = 1

        # By default, the line is marked as invalid. If the begin_point is valid,
        # we have to insert a change from invalid to valid at position 0:
        if (start_valid):
            changes.insert(0, 0)

        # Make sure the number of changes is a multiple of 2: since the line is by
        # default invalid, and has a finite part marked as valid, then the number
        # of changes should be even:
        if len(changes) % 2 != 0:
            changes.append(1)

        # The line is set to invalid by default, and this state changes for each point in the list:
        self.line_validity.reset(False, changes)

    def update_line_validity(self, match_begin : np.ndarray, match_end : np.ndarray, margin : float):
        """
        Updates the validity of the line, knowing that the section between match_begin and match_end
        should be considered valid
        
            :param match_begin: Begin point of an observed line matched with this one
            :param match_end: End point of an observed line matched with this one
            :param margin: 
        """

        # Project the match along the initial line, in order to update which
        # sections of the line are valid:

        p_begin = np.sum((match_begin - self.line_validity_begin) * self.line_validity_u)
        p_end = np.sum((match_end - self.line_validity_begin) * self.line_validity_u)
        
        # Make sure p_begin <= p_end:
        if p_begin > p_end:
            (pBegin, pEnd) = (pEnd, pBegin)

        # Update the min and max projections of the line validity:
        if p_begin < self.line_validity_min_p:
            self.line_validity_min_p = p_begin
        if p_end > self.line_validity_max_p:
            self.line_validity_max_p = p_end

        # The line should be valid between pBegin-margin and pBegin+margin:
        self.line_validity.set_value(p_begin-margin, p_end+margin, True)

    def add_valid_parts(self, dest : list['DynamicLine'], line_min_length : float, 
                        max_rho_error_sq : float, max_theta_error_sq : float, init_steps : int):
        """
        Adds to the given list the parts of this line that are valid
        
            :param dest: The list on which new line segments will be added
            :param line_min_length: Minimum length of a line
            :param max_rho_error_sq: Square of the maximum error estimate on 
                the range of a line to keep it
            :param max_theta_error_sq: Square of the maximum error estimate
                on the angle of a line to keep it
            :param init_steps: Minimum number of steps where the line is kept alive, 
                before checking the two above thresholds
        """
        
        # If the error on the position estimate of the line is too high, then we have to remove it:
        if (self.match_count > init_steps and (self.covariance[0, 0] > max_rho_error_sq 
                                               or self.covariance[1, 1] > max_theta_error_sq)):
            return

        changes = self.line_validity.splits

        # If there is only one valid section, resize this line and add it to dest:
        if len(changes) == 2:
            tmin = max(changes[0], self.line_validity_min_p)
            tmax = min(changes[1], self.line_validity_max_p)
            
            if tmax - tmin >= line_min_length:
                self.begin_point = self.line_validity_begin + tmin * self.line_validity_delta
                self.end_point = self.line_validity_begin + tmax * self.line_validity_delta

                dest.append(self)

            return

        # Otherwise, create a new line for each long enough valid section:
        for i in range(0, len(changes), 2):
            tmin = max(changes[i], self.line_validity_min_p)
            tmax = min(changes[i+1], self.line_validity_max_p)

            if tmax - tmin >= line_min_length:
                begin = self.line_validity_begin + tmin * self.line_validity_delta
                end = self.line_validity_begin + tmax * self.line_validity_delta
                dest.append(self.resized_copy(begin, end))

    def distance_of(self, point):
        """ Returns the orthogonal distance between the point and the line """

        return np.abs(point[0] * np.cos(self.state['theta']) 
                      + point[1] * np.sin(self.state['theta']) - self.state['rho'])

    def side_of_point(self, point : np.ndarray):
        """ 
        Supposing that the "forward" of the line is the vector from begin_point to end_point:
        - Returns -1 if the given point is on the left of the line
        - Returns +1 if the given point is on the right of the line
        - Returns  0 if the given point belongs to the line
        """

        # Generate a vector perpendicular to the line:
        n = self.begin_point - self.end_point
        n = np.array([-n[1], n[0]])

        # Return the sign of the dot product between (point - begin_point) and n:
        return np.sign(np.sum(n * (point - self.begin_point)))

    def check_state(self):
        """
        Check the state of the line and make sure that we have the following properties:
        * state.rho >= 0
        * 0 <= state.theta < 2*PI
        """

        # If rho < 0, rotate the line to make r >= 0 again:
        if(self.state['rho'] < 0):
            self.state['rho'] = -self.state['rho']
            self.state['theta'] += np.pi
            
            # Make sure to also update the speed to match this new representation:
            self.state['der_rho'] = -self.state['der_rho']
        
        # Make sure theta is between 0 and 2*PI:
        self.state['theta'] %= 2*np.pi

    # public Vector2 VelocityOfPoint(float x, float y) {
    #     // Perform some renamings for simplification:
    #     float rho = modelLine.state.rho;
    #     float theta = modelLine.state.theta;
    #     float dRho = modelLine.state.dRho;
    #     float dTheta = modelLine.state.dTheta;

    #     // Express the given point in the referential of the model line:
    #     float costheta = Mathf.Cos(theta), sintheta = Mathf.Sin(theta);
    #     Vector2 MP = new Vector2(x - rho * costheta, y - rho * sintheta);

    #     // Unit vectors orthogonal to the line, and along the line:
    #     Vector2 x1 = new Vector2(costheta, sintheta);       // Orthogonal
    #     Vector2 y1 = new Vector2(-sintheta, costheta);      // Along
        
    #     // MP = a.x1 + b.y1:
    #     float a = Vector2.Dot(MP, x1);
    #     float b = Vector2.Dot(MP, y1);

    #     // Compute the derivative of the point in the referential of the line:
    #     float der_along_x1 = dRho - b * dTheta;
    #     float der_along_y1 = (rho + a) * dTheta;

    #     // Express the result in the base reference:
    #     return new Vector2(
    #         der_along_x1 * costheta - der_along_y1 * sintheta,
    #         der_along_x1 * sintheta + der_along_y1 * costheta);
    # }

    def intersect_distance(self, A : np.ndarray, B : np.ndarray):
        """
        Returns the value t such that the point:
        I = line.begin_point + t * (line.end_point - line.begin_point)
        belongs to the line AB. This point is thus the intersection point
        between this line and the line AB
        """

        AB = B - A
        den = np.cross(self.end_point - self.begin_point, AB)

        if den == 0:
            return -1

        return np.cross(AB, self.begin_point - A) / den

    def __str__(self):
        return "Dynamic Line: [(%.2f, %.2f) -> (%.2f, %.2f)]: \
            rho=%.2f±%.2f; theta=%.2f±%.2f; \
            der_rho=%.2f±%.2f; der_theta=%.2f±%.2f" \
            \
        % (self.begin_point[0], self.begin_point[1], 
           self.end_point[0], self.end_point[1],
           self.state['rho'], np.sqrt(self.covariance[0,0]),
           self.state['theta'], np.sqrt(self.covariance[1,1]),
           self.state['der_rho'], np.sqrt(self.covariance[2,2]),
           self.state['der_theta'], np.sqrt(self.covariance[3,3]))
    
    def resized_copy(self, begin_point, end_point):
        """ Create a copy of this line and redefines its endpoints """

        copy = DynamicLine(0, 0, 0, begin_point, end_point)
        copy.color = self.color
        copy.state = self.state.copy()
        copy.covariance = self.covariance.copy()
        copy.match_count = self.match_count
        copy.last_match_dynamic = self.last_match_dynamic
        copy.is_static = self.is_static
        copy.check_state()

        return copy