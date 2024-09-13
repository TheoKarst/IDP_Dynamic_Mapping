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

> *match_grid*: The match grid is a grid used to register lines into cells in the geometry mapping algorithm, which allows a more efficient matching between observed lines and lines in the model
> - center_x: World position of the center of the grid along the x-axis
> - center_y: World position of the center of the grid along the y-axis
> - width: Number of cells along the x-axis
> - height: Number of cells along the y-axis
> - cell_size: Size of a cell in meters

> *geometry_extraction*: Parameters used during the extraction of primitives from the observations of the LIDAR
> - points_critical_distance: Maximum distance in meters between two consecutive points to add them to the same line
> - line_critical_distance: Maximum orthogonal distance in meters between a point and the line under extraction to add the point to the line
> - points_critical_angle: Maximum angle in degrees between two consecutive points to add them to the same line
> - circle_critical_distance: Maximum distance between a point and the center of a circle to match the point to that circle
> - line_min_length: Minimum length of a line in meters (smaller lines are considered as circle clusters)
> - line_min_points: Minimum number of points a line should have to be extracted
> - circle_min_points: Minimum number of points a circle should have to be extracted

> *geometry_matching*: Paramaters used during the matching of primitive
> - line_max_match_angle: Maximum angle in degrees between an observed line and a line in the model to have a match
> - line_max_match_orthogonal_distance: Maximum orthogonal distance between the endpoints of an observed line and a line in the model to have a match
> - line_max_match_parallel_distance: Maximum distance between the endpoints of an observed line and a line in the model, along the line from the model, to have a match
> - circle_max_match_distance: Maximum distance between the center of an observed circle and the center of a circle in the model to have a match
> - line_validity_extent: By how much the validity of an observed line should be extended when matched with a line in the model

> *dynamic_lines*: Parameters used for dynamic lines
> - use_static_lines: If true, the algorithm will use static lines instead of dynamic ones. This is more stable but less efficient to deal with dynamic objects
> - init_steps: Minimum number of steps where the line is kept alive
> - lines_max_range_error: Maximum range (rho) error we accept for a line before deleting it
> - lines_max_angle_error: Maximum angle (theta) error we accept for a line before deleting it
> - min_matches_to_consider_static: Minimum number of matches to consider an immobile line as static
> - static_max_range_derivative: Maximum value for the derivative of rho for a static line
> - static_max_angle_derivative: Maximum value for the derivative of theta for a static line
> - lines_friction: Coefficient between 0 and 1. The speed of the lines is multiplied by 1 - lines_friction at each step
> - lines_process_noise: Coefficients used to compute the process noise 4x4 matrix for the lines

> *dynamic_circles*: Parameters used for dynamic circles
> - min_distance_to_lines: Minimum distance between a circle and a line to keep the circle. This is used to prevent the creation of unwanted noisy circles near the walls
> - circles_friction: Coefficient between 0 and 1. The speed of the circles is multiplied by 1 - circles_friction at each step

> *wipe_shape*: Parameters used for the construction of the wipe shape
> - alpha: Angle in degrees used in the alpha-filtering of observations
> - epsilon: Epsilon used in Douglas-Peucker algorithm to filter observations during the construction of the wipe-shape
> - clamp_distance: Maximum distance of the points of the wipe-shape from the LIDAR
