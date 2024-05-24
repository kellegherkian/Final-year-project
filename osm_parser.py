def parse_osm_data(osm_file):
    # Initialize dictionaries to store nodes and ways
    nodes = {}
    with open(osm_file, 'r') as file:
        lines = file.readlines()

        for line in lines:
            if line.startswith('<node id='):
                # Extract node information
                node_info = line.strip().split()
                node_id = node_info[1][4:-1]  # Extracting the ID
                lat = float(node_info[2][5:-1])  # Extracting latitude
                lon = float(node_info[3][5:-3])  # Extracting longitude
                nodes[node_id] = (lat, lon)

    return nodes
