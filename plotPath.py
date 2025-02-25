import matplotlib.pyplot as plt

def serpentine_path_grid(grid):
    """Plot a serpentine path over the grid using matplotlib."""
    x_coords = []
    y_coords = []

    for i, row in enumerate(grid):
        path = row if i % 2 == 0 else reversed(row)  # Reverse every other row
        for point in path:
            lat, lon = point
            x_coords.append(lon)  # Longitude as x-axis
            y_coords.append(lat)  # Latitude as y-axis

    # Plot the path
    plt.figure(figsize=(8, 6))
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b', label='Path')
    plt.scatter(x_coords[0], y_coords[0], color='g', label='Start')
    plt.scatter(x_coords[-1], y_coords[-1], color='r', label='End')
    plt.title('Serpentine Path Grid Plot')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    plt.show()

def generate_grid(geofence, x_divisions, y_divisions):
    """Generate a search grid based on geofence coordinates."""
    print("Generating grid...")
    
    bottom_left, bottom_right, top_right, top_left = geofence
    lat_start, lon_start = bottom_left
    lat_end, lon_end = top_right

    lat_step = (lat_end - lat_start) / y_divisions
    lon_step = (lon_end - lon_start) / x_divisions

    grid = []
    for i in range(y_divisions + 1):
        row = []
        for j in range(x_divisions + 1):
            lat = lat_start + i * lat_step
            lon = lon_start + j * lon_step
            row.append((lat, lon))
        grid.append(row)
    
    return grid

# Define geofence
geofence = [
    (13.2865093, 77.5961633),  # Bottom-Left
    (13.2864995, 77.5963980),  # Bottom-Right
    (13.2868813, 77.5964610),  # Top-Right
    (13.2869165, 77.5962344)   # Top-Left
] # Change values as required

# Call the function to plot the path
grid=generate_grid(geofence, 5, 5)
serpentine_path_grid(grid)
