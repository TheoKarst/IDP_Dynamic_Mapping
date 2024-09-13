import numpy as np

import parameters.parameters as params
import geometry_maps.utils.angles as utils

from scene.robot import Robot
# from scene.scene import Scene

from geometry_maps.primitives.line_builder import LineBuilder
from geometry_maps.primitives.dynamic_line import DynamicLine
from geometry_maps.primitives.wipe_shape import WipeShape

from geometry_maps.utils.filtering import alpha_filter, douglas_peucker
from geometry_maps.utils.match_grid import MatchGrid

class GeometryMapping:
    def __init__(self, parameters : dict | None = None):
        self.parameters = parameters if parameters is not None \
            else params.geometry_params
        
        # Create a grid for a more efficient matching of primitives:
        self.match_grid = MatchGrid(
            self.parameters['match_grid']['width'],
            self.parameters['match_grid']['height'],
            self.parameters['match_grid']['cell_size'],
            self.parameters['match_grid']['center_x'],
            self.parameters['match_grid']['center_y'])
        
        # Lines and circles currently in the model:
        self.model_lines = []
        self.model_circles = []

        # Wipe shape used during the current frame, to remove inconsistent primitives:
        self.wipe_shape = None

    def update(self, robot : Robot, lidar_index : int, observations : dict, elapsed_time : float):

        # Clear the match grid and register again the current model lines:
        self.match_grid.clear()
        for line in self.model_lines:
            self.match_grid.register_line(line)

        # Compute the position and error estimate of each observation that is not 
        # out of the range of the LIDAR:
        points = self.compute_points(robot, lidar_index, observations)

        # Build the "wipe-shape" representing the free space of the LIDAR:
        self.wipe_shape = self.build_wipe_shape(robot, lidar_index, observations)

        # Extract primitives from the observed points:
        lines, circles = self.extract_primitives(points)
        
        # Update the lines in the model, using the observed lines:
        self.update_model_lines(lines, self.wipe_shape, elapsed_time)

        print("Model lines:", len(self.model_lines))

    def draw(self, scene : 'Scene'):
        if self.model_lines is not None:
            for line in self.model_lines:
                line.draw(scene)

        if self.model_circles is not None:
            for circle in self.model_circles:
                pass

        if self.wipe_shape is not None:
            self.wipe_shape.draw(scene)

        self.match_grid.draw(scene)

    def compute_points(self, robot : Robot, lidar_index : int, observations : dict):

        # Compute the position of the observations in world space:
        positions = robot.get_observations_positions(lidar_index)

        # Compute the covariance matrices representing the error estimates on these
        # positions:
        covariances = robot.get_observations_positions_covariances(lidar_index)
        
        # Create a dictionnary to represent points and remove observations that are out
        # of the range of the LIDAR:
        keep_indices = np.logical_not(observations['out_of_range'])

        points = {}
        points['angles'] = observations['angles'][keep_indices]
        points['positions'] = positions[keep_indices]
        points['covariances'] = covariances[keep_indices]

        return points
    
    def build_wipe_shape(self, robot : Robot, lidar_index : int, observations : dict):
        """
        Given the observations of a LIDAR, use these observations to build a wipe-shape
        representing the free-space of the sensor
        
            :param robot: Robot equipped with the LIDAR which made the observations
            :param lidar_index: Index of the LIDAR that made the observations
            :param observations: Observations of the LIDAR
        """
        
        # 1. Compute the global position of the observations after clamping:
        lidar_pose = robot.get_sensor_pose(lidar_index)
        ranges = np.clip(observations['ranges'], 
                         a_min=None,
                         a_max=self.parameters['wipe_shape']['clamp_distance'])
        angles = lidar_pose.angle + observations['angles']
        
        positions = np.zeros((len(ranges), 2))
        positions[:,0] = lidar_pose.x + ranges * np.cos(angles)
        positions[:,1] = lidar_pose.y + ranges * np.sin(angles)
        
        # 2. For each observation, remove the other observations in the "alpha-cone" of that observation:
        indices = alpha_filter(positions, angles, self.parameters['wipe_shape']['alpha'])
        positions = positions[indices]
        angles = angles[indices]

        # 3. Apply Douglas Peucker algorithm to the remaining points:
        keep_mask = douglas_peucker(positions, self.parameters['wipe_shape']['epsilon'])
        positions = positions[keep_mask]
        angles = angles[keep_mask]

        return WipeShape((lidar_pose.x, lidar_pose.y), positions, angles)

    def extract_primitives(self, points):
        extracted_lines = []
        extracted_circles = []

        # We first try to match the first point with a line, then with a circle.
        # If we are building a line, we should have line_builder == None.
        # If we are building a circle, we should have circle_builder == None.
        line_builder = None
        circle_builder = None

        # Last point that was added to a line:
        last_point = None

        # Get the parameters used for the extraction of primitives:
        m_params = self.parameters['geometry_extraction']
        line_process_noise = np.identity(2)     # TODO: Change this !

        for current_point in zip(points['angles'], points['positions'], points['covariances']):
            
            # Initialisation: this should be executed just for the first non null point:
            if line_builder is None and circle_builder is None:
                line_builder = LineBuilder(current_point[1], current_point[2])
                last_point = current_point
                continue

            # 1. If we are currently building a line:
            if line_builder is not None:

                # Compute the Euclidean distance between the current and the previous point,
                # the orthogonal distance between the current point and the line, and the
                # angular difference between the current and previous point:
                points_distance = np.sqrt(np.sum((current_point[1] - last_point[1]) ** 2))
                line_distance = line_builder.distance_from(current_point[1])
                angular_difference = utils.abs_delta_angles(last_point[0], current_point[0])

                # Try to match the current point with the current line:
                condition_1 = points_distance <= m_params['points_critical_distance']
                condition_2 = line_builder.points_count() < 3 \
                    or line_distance <= m_params['line_critical_distance']
                condition_3 = angular_difference <= m_params['points_critical_angle']
                
                # If the three conditions are met, we can add the point to the line:
                if condition_1 and condition_2 and condition_3:
                    line_builder.add_point(current_point[1], current_point[2])
                    last_point = current_point
                    continue

                # Else, if the current line is long enough to be extracted, then extract it, 
                # and add the current point in a new line:
                elif (line_builder.points_count() >= m_params['line_min_points']
                      and line_builder.length() >= m_params['line_min_length']):

                    extracted_lines.append(line_builder.build(line_process_noise))
                    line_builder = LineBuilder(current_point[1], current_point[2])
                    last_point = current_point
                    continue

                # Otherwise, convert the current line into a circle cluster, and continue 
                # to process this circle:
                else:
                    circle_builder = line_builder.to_circle()
                    line_builder = None

            # 2. If we are currently building a circle:

            # If the current point can be added to the current circle, add it:
            if (circle_builder.distance_from(current_point) 
                    <= m_params['circle_critical_distance']):
                
                circle_builder.add_point(current_point[1], current_point[2])

            # Otherwise, extract the current circle and add the current point in a new line:
            else:
                if circle_builder.points_count() >= m_params['circle_min_points']:
                    extracted_circles.append(circle_builder.build())

                circle_builder = None
                line_builder = LineBuilder(current_point[1], current_point[2])
                last_point = current_point

        # Finally, extract the current line or current circle if necessary:
        if (line_builder is not None 
            and line_builder.points_count() >= m_params['line_min_points'] 
            and line_builder.length() >= m_params['line_min_length']):

            extracted_lines.append(line_builder.build(line_process_noise))

        elif(circle_builder is not None 
            and circle_builder.points_count() >= m_params['circle_min_points']):
            extracted_circles.append(circle_builder.build())

        return extracted_lines, extracted_circles

    def update_model_lines(self, observed_lines : list[DynamicLine], wipe_shape : WipeShape, 
                           elapsed_time : float):
        """
        Uses the observed lines to update the lines in the model
        
            :param observed_lines: Lines extracted from the current observations
            :param wipe_shape: Wipe-shape representing the free area of the sensor
            :param elapsed_time: Elapsed time in seconds since the last update
        """

        new_lines = []

        for line in self.model_lines:
            # Reset the color of the lines from the model:
            line.color = (255, 0, 0)

            # Predict the new state of the lines, knowing the elapsed time since the last update:
            if not self.parameters['dynamic_lines']['use_static_lines']:
                # TODO: Instead of getting the process noise error from utils.params,
                # we should compute it from self.parameters:
                process_noise = params.line_process_noise_error
                lines_friction = self.parameters['dynamic_lines']['lines_friction']

                line.predict_state(elapsed_time, process_noise, lines_friction)

            # Finally, find which sections of the line are valid or not, using the wipe shape:
            wipe_shape.update_line_validity(line)

        # Try to match the observed lines with the lines in the model:
        for i in range(len(observed_lines)):
            current_line = observed_lines[i]

            # The best match we found in the model, as well as the
            # norm distance between this line and the one in the model:
            best_match = None
            min_norm_distance = -1

            m_params = self.parameters['geometry_matching']
            for model_line in self.match_grid.find_line_neighbors(current_line):
                # First test: check if the line from the model is a good match candidate:
                if (current_line.is_match_candidate(model_line,
                    m_params['line_max_match_angle'],
                    m_params['line_max_match_orthogonal_distance'],
                    m_params['line_max_match_parallel_distance'])):

                    # Second test: Compute the Mahalanobis distance between the two lines:
                    norm_distance = current_line.norm_distance_from_model(model_line)

                    if best_match is None or norm_distance < min_norm_distance:
                        best_match = model_line
                        min_norm_distance = norm_distance

            # If a match was found and is near enough, use the current line to
            # update the matched line in the model:
            if best_match is not None and min_norm_distance < 5:
                best_match.color = (0, 0, 255)  # Blue color for matched lines

                best_match.update_line_using_match(current_line,
                    self.parameters['geometry_matching']['line_validity_extent'],
                    self.parameters['dynamic_lines']['static_max_range_derivative'],
                    self.parameters['dynamic_lines']['static_max_angle_derivative'],
                    self.parameters['dynamic_lines']['min_matches_to_consider_static'])

            # Otherwise, we just add this new line to the model:
            else:
                current_line.color = (0, 255, 0)    # Green color for new lines
                new_lines.append(current_line)

        # Keep only the valid parts of the lines from the model:
        for line in self.model_lines:
            line.add_valid_parts(new_lines,
                self.parameters['geometry_extraction']['line_min_length'],
                self.parameters['dynamic_lines']['lines_max_range_error'] ** 2,
                self.parameters['dynamic_lines']['lines_max_angle_error'] ** 2,
                self.parameters['dynamic_lines']['init_steps'])

        # Replace the previous lines with the updated ones:
        self.model_lines = new_lines