import cv2
import numpy as np

# Function to calculate distance between two points
def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Function to find nearest corner to a robot
def find_nearest_corner(robot_position, corners, assigned_corners):
    min_distance = float('inf')
    nearest_corner = None
    for corner in corners:
        if corner in assigned_corners:
            continue
        dist = distance(robot_position, corner)
        if dist < min_distance:
            min_distance = dist
            nearest_corner = corner
    return nearest_corner

# Define robot positions and corners
robot_positions = [[899, 650], [558, 442], [749, 226]]
#            top         left         right
corners = [[932, 217], [759, 516], [1105, 516]]
assigned_corners = []


# Initialize frame (for visualization purpose only)
frame = np.zeros((720, 1280, 3), dtype=np.uint8)  # Assuming a frame size of 1280x720 pixels

distances = []
robots = robot_positions.copy()
min_values = []
x = []
def find_nearest_corner(corners, robots):
    corner_assigned = {}
    default=0
    dist_list = []
    robot_list=robots.copy()
    k=0
    for i in corners:
        
        for j in robots:
            dist_list.append(distance(i,j))
        # print("min dist : ",robots[dist_list.index(min(dist_list))])
        
        if len(robots)>=1:
            corner_assigned[k]=robots[dist_list.index(min(dist_list))]
            robots.pop(dist_list.index(min(dist_list)))
            dist_list.clear()
        else:
            corner_assigned[k]=robot_list
        k+=1   
        # if 
            
    
    print(corner_assigned)
    return corner_assigned
robot_position = [[899, 650], [558, 442], [749, 226]]            
dict = find_nearest_corner(corners, robot_positions)
for i in dict.keys():
    # corners[i]
    print(dict[i])
    # print("found",robot_position[robot_positions.index(dict[i])])
    cv2.line(frame, tuple(corners[i]), tuple(dict[i]), (0, 255, 0), 2)
    if dict[i] in robot_position:
        print("found",robot_position[robot_position.index(dict[i])])

print(robot_position)
print(dict.items()[0])
# for i in robots:
#     print("Robot at: ",i)
#     for j in corners:
#         distances.append(distance(i,j))
#     print("------------")
#     print(distances)
#     print(f"Min value: {min(distances)}")
#     x.append(distances.index(min(distances)))
#     min_values.append(min(distances))
#     print(f"Min distance robot: {corners[distances.index(min(distances))]}")
    
#     # x.append(distances.copy())
#     # print(x)
#     # cv2.line(frame, tuple(i), tuple(robot_positions[distances.index(min(distances))]), (0, 255, 0), 2)
#     distances.clear()
# print(x)
# print(min_values)
# print(robots)
# print(distances)


# for i, robot_position in enumerate(robot_positions):
#     nearest_corner = find_nearest_corner(robot_position, corners, assigned_corners)
#     assigned_corners.append(nearest_corner)
#     cv2.line(frame, tuple(nearest_corner), tuple(robot_position), (0, 255, 0), 2)
#     cv2.circle(frame, tuple(robot_position), 5, (0, 0, 255), -1)  # Draw a red circle at robot position

# Display the frame
cv2.imshow("Output", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()