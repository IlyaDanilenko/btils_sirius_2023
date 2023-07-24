import numpy as np
import cv2
import math
import rospy
import time
from itertools import product
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from mavros_msgs.srv import CommandBool, CommandTOL, CommandTOLRequest
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget



rospy.init_node('flight')

arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
cmd_vel_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
takeoff = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
get_map = rospy.Subscriber('rtabmap/grid_map', OccupancyGrid, callback)


cmd_vel = PositionTarget()
cmd_vel.type_mask=0b0000_01_0_111_000_111
cmd_vel.coordinate_frame=PositionTarget.FRAME_LOCAL_NED=1


rotations = ['front']
rotation_stat = 0


suc = arm(True)
if suc:
    rospy.sleep(0.2)
    suc = takeoff(CommandTOLRequest())
    if suc:
        print("ok")

rospy.sleep(5)


def filter(mtx, size): 
        filtered = deepcopy(mtx) 
        h = list(product(range(-int((size - 1) / 2), int((size - 1) / 2) + 1), repeat=2)) 
        print(h) 
        for y in range(int((size - 1) / 2), len(mtx) - int((size - 1) / 2)): 
            for x in range(int((size - 1) / 2), len(mtx[y]) - int((size - 1) / 2)): 
                ec = 0 
                for c in range(size**2): 
                    if mtx[y + h[c][0]][x + h[c][1]] == 100: 
                        ec += 1 
                if ec > (size**2 - 1) / 2: 
                    filtered[y][x] = 100 
                else: 
                    filtered[y][x] = 0
        return filtered

def check_lets(row, col):
            target = 3
            lets = {
                'front':[False, (row, col+target)],
                'rear':[False, (row, col-3)],
                'right':[False, (row + 3, col)],
                'left':[False, (row-3, col)]
            }
            for i in range(1, 7):
                if map_arr[row][col+i] == 100:
                    lets['front'][0] = True
                if map_arr[row][col-i] == 100:
                    lets['rear'][0] = True
                if map_arr[row+i][col] == 100:
                    lets['right'][0] = True
                if map_arr[row-i][col] == 100:
                    lets['left'][0] = True

            return lets['front'], lets['rear'], lets['right'], lets['left'], target

def heuristic(row, col, goal_row, goal_col):
    return math.sqrt(abs(row - goal_row)**2 + abs(col - goal_col)**2)

def get_neighbors(row, col):
    neighbors = []
    indices = [(row-1, col), (row+1, col), (row, col-1), (row, col+1), (row-1, col-1), (row-1, col+1), (row+1, col-1), (row+1, col+1)]

    for index in indices:
        if 0 <= index[0] < rows and 0 <= index[1] < cols:
            if matrix[index[0], index[1]] != 100:
                neighbors.append(index)
    
    return neighbors


def callback(data):

    global rotations
    global rotation_stat
    global cmd_vel

    map = data.data
    resolution = data.info.resolution
    start = (data.info.origin.position.x // resolution, data.info.origin.position.y // resolution)


    for point in range(len(map)):
        if map[point] == -1:
            map[point] = 0

    map_arr = np.resize((np.asarray(map)), (data.info.width, data.info.height))
    map_arr = np.asarray(filter(map_arr.tolist(), 7))

    height, width = map_arr.shape
    map_arr = cv2.rectangle(map_arr, (5, 5), (width - 5, height - 5), 100, 1)


    rows, cols = matrix.shape
    start_row, start_col = start


    row, col = start_row, start_col


    if rotation_stat == 1:

        if rotations[-1] == 'rear':
            rear, front, left, right, target = check_lets(row, col)
        elif rotations[-1] == 'right':
            left, right, front, rear, target = check_lets(row, col)
        elif rotations[-1] == 'left':
            right, left, rear, front, target = check_lets(row, col)
        else:
            front, rear, right, left, target = check_lets(row, col)

    elif rotation_stat == 2:

        if rotations[-1] == 'rear':
            right, left, rear, front, target = check_lets(row, col)
        elif rotations[-1] == 'front':
            rear, front, left, right, target = check_lets(row, col)
        elif rotations[-1] == 'left':
            front, rear, right, left, target = check_lets(row, col)
        else:
            rear, front, left, right, target = check_lets(row, col)
            

    elif rotation_stat == 3:
        if rotations[-1] == 'rear':
            rear, front, left, right, target = check_lets(row, col)
        elif rotations[-1] == 'left':
            right, left, rear, front, target = check_lets(row, col)
        elif rotations[-1] == 'right':
            left, right, front, rear, target = check_lets(row, col)
        else:
            front, rear, right, left, target = check_lets(row, col)

    else:

        front, rear, right, left, target = check_lets(row, col)


    if not front[0]:
        print('front')
        cmd_vel.velocity.x = 0.2
        cmd_vel_publisher.publish(cmd_vel) 
        rospy.sleep(target*resolution/0.2)
        cmd_vel.velocity.x = 0
        cmd_vel_publisher.publish(cmd_vel) 

        goal = front[1]

    else:

        if not right[0]:

            if rotation_stat != 3:
                rotation_stat += 1
            else:
                rotation_stat = 0
            
            cmd_vel.yaw = 0.5
            cmd_vel.velocity.x = 0
            cmd_vel.velocity.y = 0
            cmd_vel.velocity.z = 0
            cmd_vel_publisher.publish(cmd_vel) 
            rospy.sleep(3)

            cmd_vel.velocity.x = 0.1
            cmd_vel_publisher.publish(cmd_vel) 
            rospy.sleep(target*resolution/0.2)
            cmd_vel.velocity.x = 0
            cmd_vel_publisher.publish(cmd_vel) 


            print('right')
            goal = right[1]
            if rotations[-1] != 'right':
                rotations.append('right')

        elif not left[0]:

            if rotation_stat != 0:
                rotation_stat -= 1
            else:
                rotation_stat = 3

            cmd_vel.yaw = -0.5
            cmd_vel.velocity.x = 0
            cmd_vel.velocity.y = 0
            cmd_vel.velocity.z = 0
            cmd_vel_publisher.publish(cmd_vel) 
            rospy.sleep(3)

            cmd_vel.velocity.x = 0.1
            cmd_vel_publisher.publish(cmd_vel) 
            rospy.sleep(target*resolution/0.2)
            cmd_vel.velocity.x = 0
            cmd_vel_publisher.publish(cmd_vel) 

            print('left')
            goal = left[1]
            if rotations[-1] != 'left':
                rotations.append('left')

        else:

            if rotation_stat == 2:
                rotation_stat = 0

            elif rotation_stat == 3:
                rotation_stat = 1

            else:
                rotation_stat += 2

            cmd_vel.yaw = -1
            cmd_vel.velocity.x = 0
            cmd_vel.velocity.y = 0
            cmd_vel.velocity.z = 0
            cmd_vel_publisher.publish(cmd_vel) 
            rospy.sleep(5)


            cmd_vel.velocity.x = 0.1
            cmd_vel_publisher.publish(cmd_vel) 
            rospy.sleep(target*0.05/0.2)
            cmd_vel.velocity.x = 0
            cmd_vel_publisher.publish(cmd_vel) 

            goal = rear[1]
            print("Назад")
            if rotations[-1] != 'rear':
                rotations.append('rear')
        
    print(goal)
    x = 0
    while (row, col) != goal:
        x+=1
        map_arr[goal[0], goal[1]] = 150
        neighbors = get_neighbors(row, col)
        sugg_row, sugg_col = neighbors[0][0], neighbors[0][1]
        for neighbour in neighbors:
            # if len(get_neighbors(neighbour[0], neighbour[1])) == 8:
                #if neighbour not in path:
            if heuristic(sugg_row, sugg_col, goal[0], goal[1]) > heuristic(neighbour[0], neighbour[1], goal[0], goal[1]):
                sugg_row, sugg_col = neighbour[0], neighbour[1]
        map_arr[row, col] = 255
        row, col = sugg_row, sugg_col
        # cv2.imshow("filename.png", map_arr.astype(np.uint8))
        # cv2.waitKey(1)


while not rospy.is_shutdown():
    pass
