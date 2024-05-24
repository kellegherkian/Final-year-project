# Import necessary libraries and modules
import numpy as np
from car_model import CarModel  # Import a CarModel class, presumably for simulating car physics or behavior
from osm_parser import parse_osm_data  # Import a function to parse OSM (OpenStreetMap) data
from track_model import Track  # Import a Track class for modeling the track
from pygame_visualization import run_pygame, convert_to_screen  # Import functions for visualization with Pygame
# Import the calculate_max_speeds function for calculating maximum speeds based on track geometry and physics
from speed_calculator import calculate_max_speeds

def main():
    osm_file = "TrackData.txt"  # Specify the file containing OSM data for the track
    track = Track()  # Instantiate the Track object
    # Parse OSM data to get nodes, which are points on the track with latitude and longitude
    nodes = parse_osm_data(osm_file)
    print(f"Parsed {len(nodes)} nodes from OSM data")  # Print the number of parsed nodes

    # Add each node to the Track object with its latitude and longitude
    for node_id, (latitude, longitude) in nodes.items():
        track.add_node(node_id, latitude, longitude)

    # Set dimensions for the Pygame window
    screen_width = 1400
    screen_height = 800

    # Prepare track points for speed calculation and visualization by converting lat-long to tuples
    calculation_points = [(latitude, longitude) for node_id, (latitude, longitude) in nodes.items()]
    # Convert geographic coordinates to screen coordinates for visualization
    track_points_screen = convert_to_screen(calculation_points, screen_width, screen_height)

    mu = 0.8  # Coefficient of friction used in max speed calculation

    # Calculate maximum speeds at each point on the track based on physics
    max_speeds = calculate_max_speeds(calculation_points, mu)

    # Associate each track point with its calculated maximum speed for visualization
    track_data_with_speeds = [{
        'point': calculation_points[i],
        'max_speed': max_speeds[i]
    } for i in range(len(max_speeds))]

    # Print each point with its calculated maximum speed for verification
    for item in track_data_with_speeds:
        point, max_speed = item['point'], item['max_speed']
        print(f"Point: {point}, Max Speed: {max_speed:.2f} m/s")

    # Interpolate points along the centerline of the track for a smoother path
    centerline_points = track.interpolate_path(num_points=107)

    # Run the Pygame visualization with the track and speed data
    run_pygame(track_points_screen, screen_width, screen_height, track_data_with_speeds)

# Run the main function if this script is executed as the main program
if __name__ == "__main__":
    main()
