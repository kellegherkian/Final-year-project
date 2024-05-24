import numpy as np
from scipy.interpolate import interp1d

class Track:
    def __init__(self):
        """
        Initializes a Track object with an empty dictionary to store nodes.
        Each node represents a point on the track with a unique ID, latitude, and longitude.
        """
        self.nodes = {}

    def add_node(self, node_id, latitude, longitude):
        """
        Adds a node to the track.

        Parameters:
        - node_id (str): The unique identifier for the node.
        - latitude (float): The latitude of the node.
        - longitude (float): The longitude of the node.
        """
        self.nodes[node_id] = {'lat': latitude, 'lon': longitude}
        print(f"Added node {node_id}: Latitude {latitude:.6f}, Longitude {longitude:.6f}")

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        """
        Calculates the great-circle distance between two points on the earth, specified by latitude and longitude.

        Parameters:
        - lat1, lon1 (float): Latitude and longitude of the first point in degrees.
        - lat2, lon2 (float): Latitude and longitude of the second point in degrees.

        Returns:
        - distance (float): Distance between the two points in kilometers.
        """
        # Radius of the Earth in kilometers
        R = 6371.0
        # Convert latitude and longitude from degrees to radians
        lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
        # Difference in coordinates
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        # Haversine formula
        a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        distance = R * c
        return distance

    @staticmethod
    def cumulative_distance(latitudes, longitudes):
        """
        Calculates cumulative distances along a path defined by latitudes and longitudes.

        Parameters:
        - latitudes (list): List of latitudes.
        - longitudes (list): List of longitudes.

        Returns:
        - distances (list): Cumulative distances from the starting point.
        """
        distances = [0.0]
        for i in range(1, len(latitudes)):
            segment_length = Track.haversine_distance(
                latitudes[i - 1], longitudes[i - 1],
                latitudes[i], longitudes[i])
            distances.append(distances[-1] + segment_length)
        return distances

    def interpolate_path(self, num_points=107):
        """
        Interpolates the track path to create points.

        Parameters:
        - num_points (int): Number of points to interpolate along the track.

        Returns:
        - A list of tuples containing the interpolated latitude and longitude of each point.
        """
        if not self.nodes:
            print("No nodes defined in the track.")
            return []

        latitudes = [self.nodes[node]['lat'] for node in self.nodes]
        longitudes = [self.nodes[node]['lon'] for node in self.nodes]

        cumulative_distances = self.cumulative_distance(latitudes, longitudes)

        lat_interp = interp1d(cumulative_distances, latitudes)
        lon_interp = interp1d(cumulative_distances, longitudes)

        max_distance = cumulative_distances[-1]
        even_distances = np.linspace(0, max_distance, num_points)

        even_latitudes = lat_interp(even_distances)
        even_longitudes = lon_interp(even_distances)

        return list(zip(even_latitudes, even_longitudes))

    def calculate_radius_of_curvatures(self):
        """
        Calculates the radius of curvature for each set of three consecutive nodes.

        Returns:
        - A list of the radius of curvature for each applicable segment.
        """
        self.radius_of_curvatures = []
        if len(self.nodes) < 3:
            print("Not enough nodes to calculate curvature.")
            return []

        node_ids = list(self.nodes.keys())
        points = [(self.nodes[node_id]['lat'], self.nodes[node_id]['lon']) for node_id in node_ids]

        for i in range(1, len(points) - 1):
            point1, point2, point3 = points[i - 1], points[i], points[i + 1]
            radius = self.estimate_radius_of_curvature(point1, point2, point3)
            self.radius_of_curvatures.append(radius)

        return self.radius_of_curvatures

    @staticmethod
    def estimate_radius_of_curvature(point1, point2, point3):
        """
        Estimates the radius of curvature given three points.

        Parameters:
        - point1, point2, point3 (tuple): Tuples representing the coordinates of three points.

        Returns:
        - radius (float): The estimated radius of curvature.
        """
        A = np.array(point1)
        B = np.array(point2)
        C = np.array(point3)
        a = np.linalg.norm(C - B)
        b = np.linalg.norm(C - A)
        c = np.linalg.norm(B - A)
        s = (a + b + c) / 2  # Semiperimeter for Heron's formula
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        radius = (a * b * c) / (4 * area) if area != 0 else float('inf')  # Adjusted to handle straight lines
        return radius

    def get_ordered_nodes_for_calculation(self):
        """
        Retrieves the ordered nodes for calculation purposes.

        Returns:
        - A list of tuples representing the latitude and longitude of the first five nodes.
        """
        ordered_nodes = [(self.nodes[node_id]['lat'], self.nodes[node_id]['lon']) for node_id in self.nodes]
        print(f"Ordered Nodes for Calculation: {ordered_nodes[:5]}")
        return ordered_nodes
