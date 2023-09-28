import numpy as np
import matplotlib.pyplot as plt
import json

def compute_path_length(points):
    """Compute the total length of a path given a list of points."""
    return np.sum(np.sqrt(np.diff(points[:, 0])**2 + np.diff(points[:, 1])**2))

def find_point_at_distance(points, start_point, distance):
    """Find a point on a path that is a given distance away from a start point."""
    accumulated_distance = 0.0
    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i+1]
        segment_length = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        if accumulated_distance + segment_length > distance:
            ratio = (distance - accumulated_distance) / segment_length
            x = p1[0] + ratio * (p2[0] - p1[0])
            y = p1[1] + ratio * (p2[1] - p1[1])
            return [x, y]
        accumulated_distance += segment_length
    return None

def compute_perpendicular_offset(points, offset_distance):
    """Compute a line that is offset from the given line by a perpendicular distance."""
    offset_points = []
    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i+1]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        mag = np.sqrt(dx**2 + dy**2)
        dx /= mag
        dy /= mag
        perp_dx, perp_dy = -dy, dx
        offset_x = p1[0] + perp_dx * offset_distance
        offset_y = p1[1] + perp_dy * offset_distance
        offset_points.append([offset_x, offset_y])
    offset_x = p2[0] + perp_dx * offset_distance
    offset_y = p2[1] + perp_dy * offset_distance
    offset_points.append([offset_x, offset_y])
    return np.array(offset_points)

def plot_drive_from_file(logfile, relOffsetCarPath):
    # Load the JSON data
    with open(logfile, "r") as file:
        data = json.load(file)
    
    # Extracting the target path and car positions
    target_path_x, target_path_y = zip(*data['path'])
    car_positions_x, car_positions_y = zip(*[point['car_pos'] for point in data['datapoints']])
    
    # Adjust car positions to align with the last point of the target path
    delta_x_last = target_path_x[-1] - car_positions_x[0]
    delta_y_last = target_path_y[-1] - car_positions_y[0]
    adjusted_car_positions_x_last = [x + delta_x_last for x in car_positions_x]
    adjusted_car_positions_y_last = [y + delta_y_last for y in car_positions_y]
    
    # Convert target path to numpy array for processing
    path_np = np.array(data['path'])
    
    # Calculate the total length of the target path
    path_length = compute_path_length(path_np)
    
    # Find the point on the target path that is (path_length - relOffsetCarPath) units from the start
    new_start_point = find_point_at_distance(path_np, path_np[0], path_length - relOffsetCarPath)
    
    # Compute the shift required to align the car's starting position with the new start point
    shift_x = new_start_point[0] - adjusted_car_positions_x_last[0]
    shift_y = new_start_point[1] - adjusted_car_positions_y_last[0]
    
    # Adjust car positions based on the new shift
    further_adjusted_car_positions_x = [x + shift_x for x in adjusted_car_positions_x_last]
    further_adjusted_car_positions_y = [y + shift_y for y in adjusted_car_positions_y_last]
    
    # Compute the offset lines for track boundaries
    outer_border_perp = compute_perpendicular_offset(path_np, 3)
    inner_border_perp = compute_perpendicular_offset(path_np, -3)
    
    # Plotting
    plt.figure(figsize=(12, 8))

    # Plotting the target path and its borders
    plt.plot(target_path_x, target_path_y, color='gray', label='Track Path')
    plt.plot(outer_border_perp[:, 0], outer_border_perp[:, 1], color='gray', linestyle='--', alpha=0.7)
    plt.plot(inner_border_perp[:, 0], inner_border_perp[:, 1], color='gray', linestyle='--', alpha=0.7, label='Track Boundaries')

    # Plotting the adjusted car positions as a continuous line
    plt.plot(further_adjusted_car_positions_x, further_adjusted_car_positions_y, color='red', label='Car Path')

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title(f'Track Path with Boundaries vs. Adjusted Car Path (Length - {relOffsetCarPath})')
    plt.legend()
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')  # Set aspect ratio to be equal
    plt.show()

# Testing the function with the provided file and offset value of 2