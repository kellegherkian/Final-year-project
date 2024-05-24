import numpy as np
import pygame
import sys
import re
from scipy.interpolate import splprep, splev

from car_model import CarModel
from segment_simulation import get_max_speed_for_position
from speed_calculator import calculate_max_speeds, estimate_radius_of_curvature
from direction_vector import calculate_direction_vector

# Path to the track data file
track_file_path = 'TrackData.txt'




# Get user input for selecting the driving line preference
def get_driving_line_selection():
    print("Select the driving line for the car:")
    print("1: Centerline")
    print("2: Inside Line")
    print("3: Outside Line")
    choice = input("Enter your choice (1/2/3): ")
    return choice


# Calculate a point perpendicular to the line segment at a given distance
def calculate_perpendicular_point(start, end, distance, side):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    norm = np.sqrt(dx ** 2 + dy ** 2)
    if norm == 0:
        return start  # Avoid division by zero in case start and end points are the same
    dx, dy = dx / norm, dy / norm  # Normalize direction vector
    if side == 'left':
        px, py = -dy, dx  # Rotate vector to the left
    else:  # right
        px, py = dy, -dx  # Rotate vector to the right
    px, py = px * distance, py * distance  # Scale by distance
    return start[0] + px, start[1] + py  # Return the new point


# Generate points for a line parallel to the centerline at a given distance
def calculate_line_points(centerline_points, distance, side):
    line_points = []
    for i in range(len(centerline_points) - 1):
        start = centerline_points[i]
        end = centerline_points[i + 1]
        point = calculate_perpendicular_point(start, end, distance, side)
        line_points.append(point)
    # Handle the last segment
    if centerline_points:
        last_point = calculate_perpendicular_point(centerline_points[-2], centerline_points[-1], distance, side)
        line_points.append(last_point)
    return line_points


# Draw the road's edges on the screen
def draw_road(screen, centerline_points, road_width_pixels):
    left_edge_points = calculate_line_points(centerline_points, road_width_pixels / 2, 'left')
    right_edge_points = calculate_line_points(centerline_points, road_width_pixels / 2, 'right')
    pygame.draw.lines(screen, (0, 0, 0), False, left_edge_points, 2)  # Draw left edge
    pygame.draw.lines(screen, (0, 0, 0), False, right_edge_points, 2)  # Draw right edge


# Convert geographical coordinates to screen coordinates
def convert_to_screen(nodes, screen_width, screen_height, padding=25):
    points = nodes if isinstance(nodes, list) else nodes.values()
    latitudes = [point[0] for point in points]
    longitudes = [point[1] for point in points]

    # Determine the bounding box
    min_lat, max_lat = min(latitudes), max(latitudes)
    min_lon, max_lon = min(longitudes), max(longitudes)

    # Calculate scale factor
    track_width = max_lon - min_lon
    track_height = max_lat - min_lat
    scale_x = (screen_width - padding * 2) / track_width
    scale_y = (screen_height - padding * 2) / track_height
    scale = min(scale_x, scale_y)

    # Center the track on the screen
    track_center_x, track_center_y = (min_lon + max_lon) / 2, (min_lat + max_lat) / 2
    screen_center_x, screen_center_y = screen_width / 2, screen_height / 2

    # Convert to screen coordinates
    scaled_points = []
    for lat, lon in points:
        x = (lon - track_center_x) * scale + screen_center_x
        y = screen_height - ((track_center_y - lat) * scale + screen_center_y)
        scaled_points.append((int(x), int(y)))

    return scaled_points


def run_pygame(track_points, screen_width, screen_height, track_data_with_speeds):
    driving_line_selection = get_driving_line_selection()  # Get user-selected driving line
    pygame.init()
    pygame.font.init()
    info_font = pygame.font.SysFont('Arial', 18)  # Font for displaying text on the screen
    screen = pygame.display.set_mode((screen_width, screen_height))  # Create the display window
    clock = pygame.time.Clock()  # Clock used to control frame rate

    road_width_pixels = 20  # Visual width of the road in pixels
    start_time = pygame.time.get_ticks()  # Starting time of the simulation for tracking elapsed time
    finish_time = None  # Variable to store the time when the simulation is finished
    mu = 0.8  # Coefficient of friction, used in speed calculations

    nodes = {i: track_points[i] for i in range(len(track_points))}
    track_points_screen = convert_to_screen(nodes, screen_width, screen_height)
    offset_distance = road_width_pixels / 2
    inside_line_points_screen = calculate_line_points(track_points_screen, offset_distance, 'left')
    outside_line_points_screen = calculate_line_points(track_points_screen, offset_distance, 'right')
    max_speeds_center = calculate_max_speeds(track_points, mu)
    max_speeds_inside = calculate_max_speeds(inside_line_points_screen, mu)
    max_speeds_outside = calculate_max_speeds(outside_line_points_screen, mu)

    if driving_line_selection == '2':
        selected_line_points = inside_line_points_screen
        line_max_speeds = max_speeds_inside
    elif driving_line_selection == '3':
        selected_line_points = outside_line_points_screen
        line_max_speeds = max_speeds_outside
    else:
        selected_line_points = track_points_screen
        line_max_speeds = max_speeds_center

    initial_position = np.array(selected_line_points[0], dtype='float64')
    initial_velocity = np.array([1.0, 0], dtype='float64')
    car = CarModel(initial_position, initial_velocity, mass=1, max_acceleration=100, max_braking=50, tire_friction=0)
    car_index = 0

    running = True
    simulation_completed = False
    while running:
        dt = clock.get_time() / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))
        draw_road(screen, track_points_screen, road_width_pixels)

        if not simulation_completed:
            car_position = car.get_position()
            if car_index < len(selected_line_points) - 1:
                next_point = np.array(selected_line_points[car_index + 1])
                direction_vector = calculate_direction_vector(car_position, next_point)
                max_speed = get_max_speed_for_position(car_position, track_data_with_speeds, car_index)
                car.apply_acceleration(direction_vector, dt, max_speed)

                if np.linalg.norm(next_point - car_position) < 2:
                    car_index += 1

            pygame.draw.circle(screen, (0, 0, 255), (int(car_position[0]), int(car_position[1])), 5)

        if car_index >= len(selected_line_points) - 1 and not simulation_completed:
            simulation_completed = True
            if not finish_time:  # Record finish time only once
                finish_time = pygame.time.get_ticks()

            # Display speed information
        car_speed_kmh = np.linalg.norm(car.velocity) * 3.6
        speed_info_text = f"Speed: {car_speed_kmh:.2f} km/h"
        speed_info_surface = info_font.render(speed_info_text, True, (0, 0, 0))
        screen.blit(speed_info_surface, (10, 10))

        # Display timer, using finish_time if the simulation is completed
        time_to_display = finish_time if finish_time else pygame.time.get_ticks()
        elapsed_time = (time_to_display - start_time) / 1000.0
        timer_surface = info_font.render(f"Time: {elapsed_time:.2f}s", True, (0, 0, 0))
        screen.blit(timer_surface, (10, 30))

        mouse_x, mouse_y = pygame.mouse.get_pos()
        for i, point in enumerate(selected_line_points):
            pygame.draw.circle(screen, (255, 0, 0), point, 3)  # Draw the interpolated points
            distance = np.linalg.norm(np.array([mouse_x, mouse_y]) - np.array(point))
            if distance < 10:
                # Use track data speed for the hover effect
                hover_info_text = f"Track Data Speed: {line_max_speeds[i]:.2f} km/h"
                hover_info_surface = info_font.render(hover_info_text, True, (0, 0, 0))
                screen.blit(hover_info_surface, (mouse_x + 10, mouse_y + 10))
                break
        pygame.display.flip()
        clock.tick(60)
pygame.quit()