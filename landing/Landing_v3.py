#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import time

from sensor_msgs.msg import Range
from gs_vision.msg import Apriltag_array
from geometry_msgs.msg import PoseStamped

from mavros_msgs.srv import CommandBool, CommandTOL, CommandTOLRequest
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget


pose = None
delt_x, delt_y = 0, 0
speed_x, speed_y = 0, 0
height = 0

start_coords = [100, 100]



rospy.init_node('Landing_test_v2')

k_x = -1.5
k_y = 1.5
k_z = -0.1


def callback(position):
    global pose
    pose = position.pose


def distance(x1, y1, x2, y2):  # Дистанция между точками
    """Данная функция вычисляет расстояние которое должен пройти коптер \n
    по осям x and y, затем возвращает расстояние в метрах."""

    return (x2 - x1), (y2 - y1)


def converting_pixels_to_meters(range_h):  # абсолютная высота
    global height
    height = range_h.range


def p_regulator(delt_x, delt_y, h):
    """Расчет скорости с помощью п-регулятор"""

    # Так как у коптера и кратинки разные системы координае x и y меняются местами
    v_y = k_y * delt_y * h
    v_x = k_x * delt_x * h
    del_xy = math.sqrt(delt_x ** 2 + delt_y ** 2)
    v_z = k_z * h * (1 / del_xy)
    return v_x, v_y, v_z


def landing_odometry(position):
    """Посадка квадрокоптера с помощью одометрии камеры RealSense T256i"""

    global cmd_vel_publisher
    global cmd_vel
    global height

    coords_x = position.x
    coords_y = position.y

    start_coords_x = start_coords[0]
    start_coords_y = start_coords[1]

    del_x, del_y = distance(coords_x, coords_y, start_coords_x, start_coords_y)

    speed_x, speed_y, speed_z = p_regulator(delt_x, delt_y, height)
    print(f'speed_x: {speed_x} \t speed_y: {speed_y} \t speed_z: {speed_z}')
    if abs(pose.position.z) > 0.3:
        cmd_vel.velocity.z = speed_z
        cmd_vel.velocity.x = speed_x
        cmd_vel.velocity.y = speed_y
        cmd_vel_publisher.publish(cmd_vel) 
    elif abs(pose.position.z) > 0.2:
        cmd_vel.velocity.x = speed_x
        cmd_vel.velocity.y = speed_y
        cmd_vel.velocity.z = -0.25
        cmd_vel_publisher.publish(cmd_vel)
    else:
        arm(False)
        rospy.sleep(0.1)

class Pepega:
    def __init__(self, a, b):
        self.a = a
        self.b = b

takeoff = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
heings = rospy.Subscriber("/mavros/px4flow/ground_distance", Range, converting_pixels_to_meters)  # Высота полета через оптический поток(обтическая)
pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback)
arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
cmd_vel_publisher = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)


cmd_vel = PositionTarget()
cmd_vel.type_mask=0b0000_01_0_111_000_111
cmd_vel.coordinate_frame=PositionTarget.FRAME_LOCAL_NED=1


suc = arm(True)
# suc = False
if suc:
    rospy.sleep(0.2)
    suc = takeoff(CommandTOLRequest())
    if suc:
        print("ok")
rospy.sleep(5)
print('Удержание на высоте')




cmd_vel.velocity.x = 0
cmd_vel.velocity.y = 0
cmd_vel_publisher.publish(cmd_vel) 
rospy.sleep(3)

while not rospy.is_shutdown() and not flag_disarm:
    print(f'speed_x: {speed_x} \t speed_y: {speed_y}')
    if abs(pose.position.z) < 0.5:
        cmd_vel.velocity.z = -0.08 * k_z
        cmd_vel.velocity.x = speed_x
        cmd_vel.velocity.y = speed_y
    else:
        cmd_vel.velocity.z = -0.1 * k_z
        cmd_vel.velocity.x = speed_x
        cmd_vel.velocity.y = speed_y
    cmd_vel_publisher.publish(cmd_vel) 
    # rospy.sleep(0.05)

# print(covecient_y , covecient_x)

covecient_y , covecient_x = 0, 0
cmd_vel.velocity.x = 0
cmd_vel.velocity.y = 0
cmd_vel_publisher.publish(cmd_vel) 


while (abs(pose.position.z) > 0.2) and not rospy.is_shutdown():
    cmd_vel.velocity.x = 0
    cmd_vel.velocity.y = 0
    cmd_vel.velocity.z = -0.25
    cmd_vel_publisher.publish(cmd_vel)
    rospy.sleep(0.1)
    print('Высота: ', round(pose.position.z, 3))

print('Высота: ', round(pose.position.z, 3))

arm(False)
rospy.sleep(0.1)

print("Мой госпадин, посадка прошла успешно! Урааа!")