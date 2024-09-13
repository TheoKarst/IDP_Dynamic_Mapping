# Robot parameters

**Filename:** robot_params.json

> *robot_pose*: Default error estimate on the pose (x, y, angle) of the robot. This will be used if the state covariance matrix is not given in the recorded data.
> - *error_x*: Estimated error on the position of the robot along the x-axis in meters
> - *error_y*: Estimated error on the position of the robot along the y-axis in meters
> - *error_angle*: Estimated error on the angle of the robot in degrees

> *observation*:  Default error estimate on the observations (range, angle) of the LIDAR. This will be used if the observation covariance matrix is not given in the recorded data.
> - *error_range*: Estimated error on the range of an observation in meters
> - *error_angle*: Estimated error on the angle of an observation in degrees

# Mapping using geometric primitives

**Filename:** geometry_maps_params.json

> *geometry_extraction*: Parameters used during the extraction of primitives from the observations of the LIDAR
> - points_critical_distance: Maximum distance in meters between two consecutive points to add them to the same line
> - line_critical_distance: Maximum orthogonal distance in meters between a point and the line under extraction to add the point to the line
> - points_critical_angle: Maximum angle in degrees between two consecutive points to add them to the same line
> - circle_critical_distance: Maximum distance between a point and the center of a circle to match the point to that circle
> - line_min_length: Minimum length of a line in meters (smaller lines are considered as circle clusters)
> - line_min_points: Minimum number of points a line should have to be extracted
> - circle_min_points: Minimum number of points a circle should have to be extracted

> *geometry_matching*: Paramaters used during the matching of primitive
> - line_max_match_angle: 
> - line_max_match_orthogonal_distance: 
> - line_max_match_parallel_distance: 
> - circle_max_match_distance: 
> - line_validity_extent: 

> *dynamic_lines*: Parameters used for dynamic lines
> - use_static_lines: 
> - init_steps: 
> - lines_max_range_error:
> - lines_max_angle_error:
> - min_matches_to_consider_static: 
> - static_max_range_derivative: 
> - static_max_angle_derivative: 
> - lines_friction: 
> - lines_process_noise_range: 
> - lines_process_noise_angle: 
> - lines_process_noise_der_range: 
> - lines_process_noise_der_angle: 

> *dynamic_circles*: Parameters used for dynamic circles
> - min_distance_to_lines": 

> *wipe_shape*: Parameters used for the construction of the wipe shape
> - alpha: 
> - epsilon: 
> - clamp_distance: 
