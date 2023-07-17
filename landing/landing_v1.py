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


coord_center_apriltags = None
distance_apriltag = None
pose = None
covecient_x, covecient_y = 0, 0
height = 0

flag_landing = False
flag_disarm = False

camera_size = [640, 480]

rospy.init_node('Landing_test')


def callback(position):
    global pose
    pose = position.pose


def distance(x1, y1, x2, y2):  # Дистанция между точками
    global height

    pxm_x, pxm_y = px_in_m(height)
    c = math.sqrt(((x2 - x1) * pxm_x) ** 2 + ((y2 - y1) * pxm_y) ** 2)
    return c


def converting_pixels_to_meters(range_h):  # Перевод абсолютная высота
    global height

    height = range_h.range

def px_in_m(height):
    camera_vieweing_angle_horizon = 62.2 / 2
    camera_vieweing_angle = 48.8 / 2
    horizont_size = camera_size[0] / 2
    horizont_size_w = camera_size[1] / 2
    pxm_x = (math.tanh(math.radians(camera_vieweing_angle_horizon)) * height) / horizont_size
    pxm_y = (math.tanh(math.radians(camera_vieweing_angle)) * height) / horizont_size_w

    return pxm_x, pxm_y


def center_pixsels(tags):  # Выравнивание коптера по apritag  

    global coord_center_apriltags
    global distance_apriltag
    global covecient_x, covecient_y 
    global flag_landing
    global cmd_vel
    global cmd_vel_publisher
    global arm
    global flag_disarm
    global height

    if not flag_disarm:
        # print(tags.apriltags)
        if tags.apriltags:
            apriltag = tags.apriltags[0]
            
            coord_center_apriltags = [apriltag.center_x, apriltag.center_y]
            distance_apriltag = distance(coord_center_apriltags[0], coord_center_apriltags[1], camera_size[0], camera_size[1])
            covecient_x, covecient_y = vector_covecient(coord_center_apriltags)
            print("apriltag:  ", distance_apriltag)
            if distance_apriltag <= 0.3:
                flag_disarm = True
                print('Sniper: Новая траектория!')
                print('Sniper: Новая траектория!')
                print('Sniper: Новая траектория!')
                print('Sniper: Новая траектория!')
                print('Sniper: Новая траектория!')
        else:

            print(tags.apriltags)
            flag_disarm = True
            print('disarm')


def vector_covecient(coords):  #  Вычисление вектора движения
    x, y = coords
    cx = 0
    if x < camera_size[0] / 2:
        cx = -1
    elif x > camera_size[0] / 2:
        cx = 1
    cy = 0
    if y < camera_size[1] / 2:
        cy = 1
    elif y > camera_size[1] / 2:
        cy = -1

    return cx, cy


takeoff = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
pose_sub = rospy.Subscriber("/mavros/px4flow/ground_distance", Range, converting_pixels_to_meters)  # Высота полета через оптический поток(обтическая)
pose_sub2 = rospy.Subscriber("/geoscan/vision/apriltag", Apriltag_array, center_pixsels)  # поиск apriltag
pose_sub3 = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback)
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
rospy.sleep(5)

while not rospy.is_shutdown() and not flag_disarm:
    print("Кореткировка: ", covecient_x, covecient_y)
    if abs(pose.position.z) < 0.5:
        cmd_vel.velocity.z = -0.08
        cmd_vel.velocity.x = 0.05 * covecient_x
        cmd_vel.velocity.y = 0.05 * covecient_y   
    else:
        cmd_vel.velocity.z = -0.1
        cmd_vel.velocity.x = 0.1 * covecient_x
        cmd_vel.velocity.y = 0.1 * covecient_y 
    cmd_vel_publisher.publish(cmd_vel) 
    rospy.sleep(0.1)

print(covecient_y , covecient_x)

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