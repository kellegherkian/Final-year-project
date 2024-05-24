import numpy as np


def calculate_max_speeds(track_points, mu):
    """
    Calculate the maximum speed at each point on a track given the coefficient of friction.

    Parameters:
    - track_points (list of tuples): A list of points (x, y) defining the track.
    - mu (float): The coefficient of friction between the tires and the track surface.

    Returns:
    - max_speeds (list of floats): The maximum speed that can be safely maintained at each point on the track.
    """
    g = 9.81  # Acceleration due to gravity in m/s^2
    max_speeds = []

    # Loop over each set of three consecutive points on the track
    for i in range(1, len(track_points) - 1):
        point1 = track_points[i - 1]
        point2 = track_points[i]
        point3 = track_points[i + 1]

        # Estimate the radius of curvature based on these three points
        radius = estimate_radius_of_curvature(point1, point2, point3)

        # If the radius is infinite, the track is effectively a straight line at this segment
        if radius == float('inf'):
            max_speed = float('inf')  # Straight line or unable to calculate
        else:
            # Calculate the maximum speed using the formula v = sqrt(r * g * mu)
            max_speed = np.sqrt(radius * g * mu)

        max_speeds.append(max_speed)

    return max_speeds


def estimate_radius_of_curvature(point1, point2, point3):
    """
    Estimates the radius of curvature given three points on the track.

    Parameters:
    - point1, point2, point3 (tuple): Three consecutive points on the track.

    Returns:
    - radius (float): The estimated radius of curvature. Returns float('inf') if points are collinear.
    """
    # Convert points to numpy arrays for easier mathematical operations
    A, B, C = np.array(point1), np.array(point2), np.array(point3)

    # Calculate the midpoints of segments AB and BC
    D = (A + B) / 2
    E = (B + C) / 2

    # Helper function to calculate the slope of a line segment
    def slope(P1, P2):
        if P2[0] - P1[0] == 0:  # Avoid division by zero for vertical lines
            return None
        return (P2[1] - P1[1]) / (P2[0] - P1[0])

    slope_AB = slope(A, B)
    slope_BC = slope(B, C)

    # Helper function to calculate the slope of a perpendicular bisector
    def perpendicular_slope(s):
        if s is None:  # Horizontal line
            return 0
        if s == 0:  # Vertical line
            return None
        return -1 / s

    slope_perp_AB = perpendicular_slope(slope_AB)
    slope_perp_BC = perpendicular_slope(slope_BC)

    # Helper function to get the coefficients of a line equation in Ax + By = C form
    def line_equation(P, slope):
        if slope is None:  # Vertical line
            return 1, 0, P[0]
        A = -slope
        B = 1
        C = -slope * P[0] + P[1]
        return A, B, C

    # Get line equation coefficients for the perpendicular bisectors
    A1, B1, C1 = line_equation(D, slope_perp_AB)
    A2, B2, C2 = line_equation(E, slope_perp_BC)

    # Solve the system of linear equations to find the intersection point (center of the circumcircle)
    A = np.array([[A1, B1], [A2, B2]])
    C = np.array([C1, C2])

    try:
        center = np.linalg.solve(A, C)
    except np.linalg.LinAlgError:
        # If the points are collinear or the system is otherwise unsolvable, assume a straight path
        return float('inf')  # Infinite radius of curvature implies a straight line

    # Calculate the radius as the distance from the center to any of the points (using point A here)
    radius = np.linalg.norm(center - A)

    return radius
