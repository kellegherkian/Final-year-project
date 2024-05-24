
import numpy as np

def calculate_direction_vector(current_position, next_position):
    direction = np.array(next_position) - np.array(current_position)
    return direction / np.linalg.norm(direction) if np.linalg.norm(direction) > 0 else np.array([0.0, 0.0])
