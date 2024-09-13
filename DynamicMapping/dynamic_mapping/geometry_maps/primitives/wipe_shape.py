import numpy as np

from geometry_maps.primitives.dynamic_line import DynamicLine

class WipeShape:
    def __init__(self, center : tuple, positions : list | np.ndarray, angles : list | np.ndarray):
        """
        Instantiates a wipe-shape, that is a polygon representing the free space
        around the sensor
        
            :param center: Point inside the polygon, such that each line between the center
                and a point of the polygon should belong to the polygon. The polygon is thus
                supposed to be a "radial set", which is the case if the center is the position
                of the LIDAR
            :param positions: Positions of the points of the polygon
            :param angles: Angles of the rays between the center and the positions
        """

        self.center = center
        self.positions = positions
        self.angles = angles

    def draw(self, scene : 'Scene'):
        """ Draws the wipe shape in the scene """

        scene.draw_polygon(self.positions, (0, 255, 0), 1)

    def update_line_validity(self, line : DynamicLine):
        """
        Updates which part of the given line are valid or invalid, knowing that 
        all the parts overlapping the wipe-shape should be invalid
        """
        
        assert len(self.positions) >= 3, "Wipe-shape has less than three points !"

        # All the indices are modulo n:
        n = len(self.positions)

        # Compute if we have to check the intersections between the edges of the
        # shape and the line in clockwise or counter-clockwise order:
        if line.side_of_point(self.center) >= 0:
            # Turn clockwise:
            direction = -1

            start_section = self.find_section_upper(line.begin_point)
            end_section = self.find_section_lower(line.end_point)
            if (end_section < 0):
                end_section += n
            count = start_section - end_section

        else:
            # Turn counter-clockwise:
            direction = 1

            start_section = self.find_section_lower(line.begin_point)
            end_section = self.find_section_upper(line.end_point)
            if (start_section < 0):
                start_section += n;
            count = end_section - start_section

        # We have startPoint in [0; n[ and endPoint in [0; n[. Thus count is in ]-n; n[. 
        # We just have to keep count in ]0; n]:
        if count <= 0:
            count += n

        index = start_section
        current = self.positions[index]
        is_right = line.side_of_point(current) >= 0

        # If the start point of the line is outside the shape, and if the current
        # side of the points of the shape compared to the line is known or not.
        # The side is said unknown if the points belongs to the line. The initial values
        # doesn't matter since they will be replaced during the first loop:
        start_point_outside = True
        current_side_unknown = True

        intersections = []
        for i in range(count):
            index += direction

            # Keep indices modulo n:
            if index < 0:
                index += n
            elif index >= n:
                index -= n

            next = self.positions[index]
            next_line_side = line.side_of_point(next)
            
            # Here we manage the situation where the point is exactly on the line.
            # In this case, we keep the same state as before:
            next_is_right = is_right if next_line_side == 0 else next_line_side > 0

            # Compute start_point_outside:
            if (i == 0):
                # Rotate (next - current) to point outside the shape:
                normal = current - next
                normal = direction * np.array([-normal[1], normal[0]])
                
                # Compute the dot product between (line.begin_point - current) and the normal:
                dot_n = np.sum((line.begin_point - current) * normal)

                # If the dot product is strictly greater than 0, the point is outside the shape:
                start_point_outside = dot_n > 0

                # But if the dot product equals zero (this can happen if line.start == current),
                # the point is on the line, and we mark the current side as unknown:
                current_side_unknown = dot_n == 0

            # There may be an intersection only if current and next are on different sides of the line:
            if next_is_right != is_right and not current_side_unknown:
                # Compute the distance between the intersection and the begin point of the line.
                # The returned distance should be between 0 (intersection == begin_point)
                # and 1 (intersection == end_point):
                distance = line.intersect_distance(current, next)
                if distance > 0 and distance < 1:

                    # The distance between two changes should be greater than 0.01:
                    if len(intersections) > 0 and distance - intersections[-1] < 0.01:
                        intersections.pop()
                    else:
                        intersections.append(distance)

            # If all the previous points were exactly on the line, but not the next point,
            # we can update the value of startPointOutside and current_side_unknown:
            if current_side_unknown and next_line_side != 0:
                # If direction == -1, we know the center of the shape is on the right of the
                # line, and if nextLineDistance > 0, we know that the next point is also on
                # the right of the line. In this situation, the line is going outside of the
                # shape, so startPointOutside = true.
                # This is also the case if direction == 1 and nextLineDistance < 0:
                start_point_outside = (direction == -1 and next_line_side > 0) \
                                    or (direction == 1 and next_line_side < 0)
                current_side_unknown = False

            current = next
            is_right = next_is_right

        line.reset_line_validity(start_point_outside, intersections)

    def find_section_upper(self, point : np.ndarray):
        """
        Returns the index of a point from the shape, with an angle greater or
        equal to the given point
        """

        point_angle = np.arctan2(point[1] - self.center[1], point[0] - self.center[0]);

        # Make sure that the angle is greater than the angle of the first point 
        # of the shape:
        point_angle = self.angles[0] + (point_angle - self.angles[0]) % (2*np.pi)
        
        # self.angles[section-1] < point_angle <= self.angles[section]:
        section = np.searchsorted(self.angles, point_angle, side='left')

        return 0 if section == len(self.angles) else section

    def find_section_lower(self, point : np.ndarray):
        """
        Return the index of a point from the shape, with an angle less or 
        equal to the given point
        """

        point_angle = np.arctan2(point[1] - self.center[1], point[0] - self.center[0])

        # Make sure that the angle is greater than the angle of the
        # first point of the shape:
        point_angle = self.angles[0] + (point_angle - self.angles[0]) % (2*np.pi)

        # self.angles[section] <= point_angle < self.angles[section+1]:
        section = np.searchsorted(self.angles, point_angle, side='right') - 1

        return len(self.angles) - 1 if section == -1 else section