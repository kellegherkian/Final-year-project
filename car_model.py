import numpy as np
from direction_vector import calculate_direction_vector

class CarModel:
    def __init__(self, initial_position, initial_velocity, mass, max_acceleration, max_braking, tire_friction):
        # Initialize the car model with starting parameters.
        self.position = initial_position  # Current position of the car
        self.velocity = initial_velocity  # Current velocity vector of the car
        self.mass = mass  # Mass of the car (not currently used in this simple model)
        self.max_acceleration = max_acceleration  # Maximum acceleration the car can achieve
        self.max_braking = max_braking  # Maximum deceleration (braking power) of the car
        self.tire_friction = tire_friction  # Tire friction coefficient (not currently used)

    def apply_throttle(self, throttle, dt):
        # Apply acceleration to the car based on throttle input.
        acceleration = throttle * self.max_acceleration
        self.velocity += acceleration * dt  # Increase velocity based on acceleration
        self.update_position(dt)  # Update car's position

    def apply_brake(self, brake, dt):
        # Apply braking to the car based on brake input.
        deceleration = brake * self.max_braking
        velocity_magnitude = np.linalg.norm(self.velocity)
        # Calculate new speed after applying deceleration
        new_velocity_magnitude = max(0, velocity_magnitude - deceleration * dt)
        # Apply the deceleration or set velocity to zero if it would reverse the direction
        self.velocity = (self.velocity / velocity_magnitude * new_velocity_magnitude
                         if velocity_magnitude > 0 else np.array([0.0, 0.0]))
        self.update_position(dt)  # Update car's position

    def update(self, dt, max_speed):
        # Update the car's velocity and position for the simulation timestep.
        current_speed = np.linalg.norm(self.velocity)
        # If current speed exceeds max speed, throttle back to max speed
        if current_speed > max_speed:
            self.velocity = (self.velocity / current_speed) * max_speed
        self.update_position(dt)  # Update car's position

    def apply_acceleration(self, direction_vector, dt, max_speed):
        # Accelerate the car in the specified direction, without exceeding max speed.
        potential_velocity = self.velocity + self.max_acceleration * np.array(direction_vector) * dt
        # If the new velocity is greater than max speed, cap it at max speed
        if np.linalg.norm(potential_velocity) > max_speed:
            potential_velocity = potential_velocity / np.linalg.norm(potential_velocity) * max_speed
        self.velocity = potential_velocity
        self.update_position(dt)  # Update car's position

    def update_position(self, dt):
        # Update the car's position based on the current velocity.
        self.position += self.velocity * dt

    def set_position(self, position):
        # Set the car's position.
        self.position = position

    def get_position(self):
        # Return the car's current position.
        return self.position

