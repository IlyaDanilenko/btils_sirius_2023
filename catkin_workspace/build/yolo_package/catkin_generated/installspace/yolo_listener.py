#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def yolo_listener():
    rospy.init_node('yolo_listener', anonymous=True)
    rospy.Subscriber('yolo_pub', String, callback)
    rospy.spin()


if __name__ == '__main__':
    yolo_listener()