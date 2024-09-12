import numpy as np

def alpha_filter(positions : np.ndarray, angles : np.ndarray, alpha : float):
    """
    For each observation of a LIDAR, remove all the observations in the "alpha-cone" of that
    observation.

        :param positions: List of positions of the observations
        :param angles: List of angles of the observations from the LIDAR
        :param alpha: Angle of the "alpha-cone"
        :returns: The indices of the observations to keep after filtering
    """
    # Register each point that should be deleted:
    remove = np.zeros(len(positions))
    
    # For each point, remove all the points in the "alpha-cone" of this point:
    for i in range(len(positions)):
        if (remove[i]):
            continue

        # 1. Remove points in the cone in counter-clockwise order, between theta and theta + alpha:
        index = i
        theta = angles[i]
        stop_angle = theta + alpha

        # cos(x + PI/2) = -sin(x); sin(x + PI/2) = cos(x)
        u = np.array([-np.sin(stop_angle), np.cos(stop_angle)])

        while (theta < stop_angle):
            index += 1

            # If we reached the last point, return to the first one:
            if (index >= len(positions)):
                index = 0
                stop_angle -= 2 * np.pi

            theta = angles[index]
            dot_u = np.sum((positions[index] - positions[i]) * u)

            if (dot_u <= 0):
                remove[index] = True
            else:
                break

        # 2. Remove points in the cone in clockwise order, between theta and theta - alpha:
        index = i
        theta = angles[i]
        stop_angle = theta - alpha

        # cos(x - PI/2) = sin(x); sin(x - PI/2) = -cos(x)
        u = np.array([np.sin(stop_angle), -np.cos(stop_angle)])

        while (theta > stop_angle):
            index -= 1

            # If we reached the first point, return to the last one:
            if (index <= 0):
                index = len(positions) - 1
                stop_angle += 2 * np.pi

            theta = angles[index]
            dot_u = np.sum((positions[index] - positions[i]) * u)

            if (dot_u <= 0):
                remove[index] = True
            else:
                break

    # Return the indices of the points to keep:
    return np.nonzero(remove == False)

def douglas_peucker(positions : np.ndarray, epsilon : float):
    """
    Run Douglas-Peucker algorithm on the given positions between start (inclusive)
    and end (inclusive)
    
        :param positions: Positions of the points of the shape to filter
        :param epsilon: Parameter of the algorithm that defines how much the curve will
            be filtered
        :returns: The mask of positions to keep
    """

    # If there are less than two points, we always keep them:
    if len(positions) <= 2:
        return np.ones(len(positions), dtype=bool)

    stack = []
    stack.append([0, len(positions)-1])

    # Mask of positions to remove:
    remove = np.zeros(len(positions), dtype=bool)

    while stack:
        start, end = stack.pop()
        start_point, end_point = positions[start], positions[end]

        if end - start <= 1:
            continue

        # Create a unit vector orthogonal to the line between start_point and end_point:
        u = np.array([start_point[1] - end_point[1], end_point[0] - start_point[0]])
        u /= np.sqrt(np.sum(u ** 2))

        # For each point between start (exclusive) and end (exclusive), compute the
        # orthogonal distance between that point and the line [start_point, end_point]:
        distances = np.abs(np.sum((positions[start+1:end] - start_point) * u, axis=1))

        # Ignore points that have already been deleted:
        distances[remove[start+1:end]] = 0

        # Find the point with the maximum orthogonal distance:
        index = np.argmax(distances)
        max_distance = distances[index]

        if max_distance > epsilon:
            index += start + 1
            stack.append([start, index])
            stack.append([index, end])
        else:
            remove[start+1:end] = True
    
    # Return the mask of positions to keep after filtering:
    return remove == False