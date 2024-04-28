import cv2
import numpy as np
from queue import PriorityQueue

# Function to calculate distance between two points
def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Function to detect ArUco markers
def detect_markers(frame):
    # Your ArUco marker detection code here
    # This function should return the IDs and corners of detected markers
    return marker_ids, marker_corners

# Function to track robots
def track_robots(frame):
    # Your robot tracking code here
    # This function should return the positions of the robots
    return robot_positions
def centroid_of_triangle(corner1, corner2, corner3):
    return [(corner1[0] + corner2[0] + corner3[0]) / 3, (corner1[1] + corner2[1] + corner3[1]) / 3]

# Function to find nearest corner to a robot
def find_farthest_corner(centroid, corners, assigned_corners):
    max_distance = 0
    farthest_corner = None
    for corner in corners:
        if corner in assigned_corners:
            continue
        dist = distance(centroid, corner)
        if dist > max_distance:
            max_distance = dist
            farthest_corner = corner
    return farthest_corner

# A* pathfinding algorithm
def astar(start, goal, obstacles, grid_size):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {tuple(position): float('inf') for position in obstacles}
    g_score[tuple(start)] = 0
    f_score = {tuple(position): float('inf') for position in obstacles}
    f_score[tuple(start)] = distance(start, goal)

    while not open_set.empty():
        current = open_set.get()[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for neighbor in [(0, -grid_size), (0, grid_size), (-grid_size, 0), (grid_size, 0)]:
            neighbor_position = (current[0] + neighbor[0], current[1] + neighbor[1])

            if neighbor_position not in obstacles:
                continue

            tentative_g_score = g_score[tuple(current)] + distance(current, neighbor_position)

            if tentative_g_score < g_score.get(tuple(neighbor_position), float('inf')):
                came_from[tuple(neighbor_position)] = current
                g_score[tuple(neighbor_position)] = tentative_g_score
                f_score[tuple(neighbor_position)] = tentative_g_score + distance(neighbor_position, goal)
                open_set.put((f_score[tuple(neighbor_position)], tuple(neighbor_position)))

    return None

# Main loop
# frame = np.zeros((720, 1280, 3), dtype=np.uint8)
frame = cv2.imread(r"computerVision/map.png") 
        # Assign corners to robots
robot_positions = [[899, 650], [558, 442], [749, 226]]
#            top         left         right
corners = [[932, 217], [759, 516], [1105, 516]]
assigned_corners = []
centroid = centroid_of_triangle(corners[0], corners[1], corners[2])
# for robot_position in robot_positions:
#     nearest_corner = find_nearest_corner(robot_position, corners, assigned_corners)
#     assigned_corners.append(nearest_corner)
#     # Move the robot to its assigned corner (you need to implement this part)
#     # Draw a line from the corner to the robot position (for visualization)
#     cv2.line(frame, tuple(nearest_corner), tuple(robot_position), (0, 255, 0), 2)

# for robot_position in robot_positions:
#     farthest_corner = find_farthest_corner(centroid, corners, assigned_corners)
#     assigned_corners.append(farthest_corner)
#     # Move the robot to its assigned corner (you need to implement this part)
#     # Draw a line from the corner to the robot position (for visualization)
#     cv2.line(frame, tuple(farthest_corner), tuple(robot_position), (0, 255, 0), 2)
        # A* pathfinding for each robot to their assigned corner
for robot_position in robot_positions:
    nearest_corner = min(corners, key=lambda corner: distance(robot_position, corner))
    assigned_corners.append(nearest_corner)
    # Perform A* pathfinding to find the shortest path for the robot to the assigned corner
    path = astar(robot_position, nearest_corner, robot_positions + [corners[0], corners[1], corners[2]], grid_size=10)
    if path:
        # Move the robot along the path (you need to implement this part)
        # Draw the path (for visualization)
        for i in range(len(path) - 1):
            cv2.line(frame, path[i], path[i + 1], (0, 255, 0), 2)


cv2.imshow("Output", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
