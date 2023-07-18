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
delt_x, delt_y = 0, 0
speed_x, speed_y = 0, 0
height = 0
k_z = 1

flag_landing = False
flag_disarm = False

camera_size = [640, 480]

rospy.init_node('Landing_test_v2')

k_x = -1.5
k_y = 1.5


def callback(position):
    global pose
    pose = position.pose


def distance(x1, y1, x2, y2):  # Дистанция между точками
    """Данная функция вычисляет расстояние которое должен пройти коптер \n
    по осям x and y, затем возвращает расстояние в метрах."""

    pxm_x, pxm_y = px_in_m(height)
    return ((x2 - x1) * pxm_x) * 1, ((y2 - y1) * pxm_y) * 1


def converting_pixels_to_meters(range_h):  # Перевод абсолютная высота
    global height

    height = range_h.range

def px_in_m(height):
    """Данная функция переводит пиксеили в метры (как бы странной это не звучало) \n
    Для перевода мы используем параметры камеры: угол обзора по x and y, \n
    разрешение камеры. И также высоту полета коптера. \n
    Используя немного геометрии мы получаем нужные нам значения."""

    camera_vieweing_angle_horizon = 62.2 / 2
    camera_vieweing_angle = 48.8 / 2
    horizont_size = camera_size[0] / 2
    horizont_size_w = camera_size[1] / 2
    pxm_x = (math.tanh(math.radians(camera_vieweing_angle_horizon)) * height) / horizont_size
    pxm_y = (math.tanh(math.radians(camera_vieweing_angle)) * height) / horizont_size_w

    return pxm_x, pxm_y


def alignment_by_apritag(tags):  # Выравнивание коптера по apritag  
    """Вызывается при использовании ноды с распознаванием april tag \n
    и обрабатывает полученные результаты. \n
    Итог: расчет скорости коптера для корректной посадки на apri tag."""

    global coord_center_apriltags
    global speed_x
    global speed_y
    global flag_landing
    global flag_disarm
    global k_z

    if not flag_disarm:
        # print(tags.apriltags)
        # print(height)
        if tags.apriltags:
            apriltag = tags.apriltags[0]
            
            coord_center_apriltags = [apriltag.center_x, apriltag.center_y]
            delt_x, delt_y = distance(coord_center_apriltags[0], coord_center_apriltags[1], camera_size[0] / 2, camera_size[1] / 2)
            # print('\t', delt_x, delt_y, camera_size[0] / 2, camera_size[1] / 2)
            # print(coord_center_apriltags)
            speed_x, speed_y = p_regulator(delt_x, delt_y, height)
            # print(speed_x, speed_y)
            k_z = 1
            if (abs(pose.position.z) < 0.4):
                flag_disarm = True
                print('Новая траектория...')   
        else:
            k_z = -1
            if (abs(pose.position.z) > 1.8):
                flag_disarm = True
                print('Новая траектория...')
            speed_x, speed_y = 0., 0.
            print('waiting for tags...')


def p_regulator(delt_x, delt_y, height):
    """Расчет скорости с помощью п-регулятор"""

    # Так как у коптера и кратинки разные системы координае x и y меняются местами
    v_y = k_y * delt_y #* height
    v_x = k_x * delt_x #* height
    return v_x, v_y


takeoff = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
pose_sub = rospy.Subscriber("/mavros/px4flow/ground_distance", Range, converting_pixels_to_meters)  # Высота полета через оптический поток(обтическая)
pose_sub2 = rospy.Subscriber("/geoscan/vision/apriltag", Apriltag_array, alignment_by_apritag)  # Распознование apriltag
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