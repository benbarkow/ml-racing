import json
import matplotlib.pyplot as plt
import numpy as np

def compute_path_length(points):
    """Compute the total length of a path given a list of points."""
    return np.sum(np.sqrt(np.diff(points[:, 0])**2 + np.diff(points[:, 1])**2))

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

def find_nearest_point_on_path(point, path_points):
    """Find the nearest point on a path to a given point."""
    distances = np.sum((path_points - point)**2, axis=1)
    return path_points[np.argmin(distances)]

def compute_tangent_at_point(point, path_points):
    """Compute the tangent direction at a point on a path."""
    # Find the nearest point on the path
    nearest_point = find_nearest_point_on_path(point, path_points)
    idx = np.where((path_points == nearest_point).all(axis=1))[0][0]
    if idx == 0:
        next_point = path_points[idx + 1]
        tangent = next_point - nearest_point
    elif idx == len(path_points) - 1:
        prev_point = path_points[idx - 1]
        tangent = nearest_point - prev_point
    else:
        prev_point = path_points[idx - 1]
        next_point = path_points[idx + 1]
        tangent = next_point - prev_point
    # Normalize the tangent
    tangent /= np.linalg.norm(tangent)
    return tangent

def plot_drive(json_file):
    # Load the JSON data
    with open(json_file, "r") as file:
        data = json.load(file)
    
    # Extracting the target path, car positions, and car orientations
    target_path_x, target_path_y = zip(*data['path'])
    car_positions = [point['car_pos'] for point in data['datapoints']]
    car_orientations = [point['car_direction'] for point in data['datapoints']]
    
    # Convert target path to numpy array for processing
    path_np = np.array(data['path'])
    
    # Getting the initial car position
    init_car_pos = np.array(car_positions[0])
    
    # Adjust the path and car positions so that the initial car position becomes (0, 0)
    path_np -= init_car_pos
    car_positions = np.array(car_positions) - init_car_pos
    
    # Compute the offset lines for track boundaries
    outer_border_perp = compute_perpendicular_offset(path_np, 3)
    inner_border_perp = compute_perpendicular_offset(path_np, -3)
    
    # Plotting
    plt.figure(figsize=(12, 8))

    # Plotting the target path and its borders
    plt.plot(outer_border_perp[:, 0], outer_border_perp[:, 1], color='gray', linestyle='--', alpha=0.7)
    plt.plot(inner_border_perp[:, 0], inner_border_perp[:, 1], color='gray', linestyle='--', alpha=0.7, label='Track Boundaries')

    # Plot the car's path using a continuous line
    car_positions_x, car_positions_y = zip(*car_positions)
    plt.plot(car_positions_x, car_positions_y, color='red', label='Car Path')

    # Use arrows to indicate the car's orientation with dynamic interval based on angle
    i = 0
    while i < len(car_positions):
        pos = car_positions[i]
        orientation = car_orientations[i]
        tangent = compute_tangent_at_point(pos, path_np)
        
        # Compute angle between car's orientation and path's tangent (in degrees)
        dot_product = np.dot(orientation, -tangent)
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0)) * (180 / np.pi)
        
        # Adjust the interval based on the angle
        interval = max(1, int((90 - angle) / 90 * 10))
        # plt.arrow(pos[0], pos[1], orientation[0], orientation[1], 
        #           head_width=3, head_length=3, fc='blue', ec='blue')

        # Compute the end point of the line segment
        line_end_x = pos[0] + orientation[0]*5
        line_end_y = pos[1] + orientation[1]*5

        # Draw the line segment
        plt.plot([pos[0], line_end_x], [pos[1], line_end_y], color='blue')

        # Draw the arrow head at the end of the line segment
        plt.arrow(line_end_x, line_end_y, orientation[0]*0.1, orientation[1]*0.1, 
                    head_width=1, head_length=2, fc='blue', ec='blue')

        i += interval

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Track Path with Boundaries vs. Car Path and Orientations')
    plt.legend()
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')  # Set aspect ratio to be equal
    plt.show()

