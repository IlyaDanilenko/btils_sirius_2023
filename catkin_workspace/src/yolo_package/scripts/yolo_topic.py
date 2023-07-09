#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("cv_camera_node/image_raw", Image, self.callback)

  def callback(self,data):
    rospy.loginfo("Kek")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
      rospy.loginfo(str(e))

    (rows,cols,channels) = cv_image.shape

    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as e:
      rospy.loginfo(str(e))

def main():
  rospy.init_node('image_converter')
  ic = ImageConverter()
  try:
    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")
    
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()