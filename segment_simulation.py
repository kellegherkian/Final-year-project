import numpy as np
def get_max_speed_for_position(car_position, track_data_with_speeds, current_index):
    """
    Finds the maximum allowable speed for the car's current position on the track.

    :param car_position: The current position of the car on the track.
    :param track_data_with_speeds: List of dicts containing track points with their max speeds.
    :param current_index: The current index of the car on the track.
    :return: The maximum allowable speed for the current position.
    """
    car_pos_array = np.array(car_position)  # Convert car position to NumPy array

    # Determine a search range around the current index to find the closest track point
    search_radius = 10  # Define the number of points to look ahead and behind
    start_index = max(current_index - search_radius, 0)  # Start of the search range
    end_index = min(current_index + search_radius, len(track_data_with_speeds))  # End of the search range

    min_distance = float('inf')  # Initialize minimum distance to infinity
    associated_max_speed = None  # Initialize associated max speed

    # Iterate over the track points within the search range to find the closest point
    for i in range(start_index, end_index):
        data = track_data_with_speeds[i]
        track_point = np.array(data['point'])  # Get the track point position
        max_speed = data['max_speed']  # Get the max speed at this track point
        distance = np.linalg.norm(car_pos_array - track_point)  # Calculate distance from car to track point

        # If this track point is closer than any previous ones, update min distance and max speed
        if distance < min_distance:
            min_distance = distance
            associated_max_speed = max_speed

    # Return the max speed associated with the closest track point
    return associated_max_speed if associated_max_speed is not None else 0
