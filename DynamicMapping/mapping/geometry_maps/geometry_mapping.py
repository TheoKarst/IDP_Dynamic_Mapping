import numpy as np
from robot import Robot
from .line_builder import LineBuilder
from .dynamic_line import DynamicLine
import parameters.parameters as params
import utils

class GeometryMapping:
    def __init__(self, parameters : dict | None = None):
        self.last_time_update = -1

        self.parameters = parameters if parameters is not None \
            else params.geometry_params
        
        self.lines = None
        self.circles = None

    def update(self, robot : Robot, lidar_index : int, observations : dict, current_time : float):

        # Compute the elapsed time since the last update:
        delta_time = 0 if self.last_time_update == -1 else current_time - self.last_time_update
        self.last_time_update = current_time

        # Compute the position and error estimate of each observation that is not 
        # out of the range of the LIDAR:
        points = self.compute_points(robot, lidar_index, observations)

        # Extract primitives from the observed points:
        self.lines, self.circles = self.extract_primitives(points)
        
        # print("Lines: %i,\tCircles: %i" % (len(lines), len(circles)))

    def draw(self, scene : 'Scene'):
        if self.lines is not None:
            for line in self.lines:
                line.draw(scene)

        if self.circles is not None:
            for circle in self.circles:
                pass

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
        params = self.parameters['geometry_extraction']
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
                condition_1 = points_distance <= params['points_critical_distance']
                condition_2 = line_builder.points_count() < 3 \
                    or line_distance <= params['line_critical_distance']
                condition_3 = angular_difference <= params['points_critical_angle']
                
                # If the three conditions are met, we can add the point to the line:
                if condition_1 and condition_2 and condition_3:
                    line_builder.add_point(current_point[1], current_point[2])
                    last_point = current_point
                    continue

                # Else, if the current line is long enough to be extracted, then extract it, 
                # and add the current point in a new line:
                elif (line_builder.points_count() >= params['line_min_points']
                      and line_builder.length() >= params['line_min_length']):

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
                    <= params['circle_critical_distance']):
                
                circle_builder.add_point(current_point[1], current_point[2])

            # Otherwise, extract the current circle and add the current point in a new line:
            else:
                if circle_builder.points_count() >= params['circle_min_points']:
                    extracted_circles.append(circle_builder.build())

                circle_builder = None
                line_builder = LineBuilder(current_point[1], current_point[2])
                last_point = current_point

        # Finally, extract the current line or current circle if necessary:
        if (line_builder is not None 
            and line_builder.points_count() >= params['line_min_points'] 
            and line_builder.length() >= params['line_min_length']):

            extracted_lines.append(line_builder.build(line_process_noise))

        elif(circle_builder is not None 
            and circle_builder.points_count() >= params['circle_min_points']):
            extracted_circles.append(circle_builder.build())

        return extracted_lines, extracted_circles