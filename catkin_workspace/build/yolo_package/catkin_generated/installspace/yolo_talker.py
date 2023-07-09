#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String


def yolo_talker():
    pub = rospy.Publisher('yolo_pub', String, queue_size=10)
    
    rospy.init_node('yolo_talker', anonymous=True)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        hello_str = 'YOLO opened!'
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        yolo_talker()

    except rospy.ROSInterruptException:
        pass