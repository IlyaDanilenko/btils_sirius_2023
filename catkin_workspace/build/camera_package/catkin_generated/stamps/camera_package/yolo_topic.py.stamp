#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch
import os

class ImageConverter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("cv_camera_node/image_raw", Image, self.callback)
    self.model = torch.hub.load(rospy.get_param(rospy.search_param('yolo_model')), 'custom', 'boxes_best.pt', source='local', force_reload=True)


  def find_boxes(self, image):
    results = self.model(image)
    stride, names, pt = self.model.stride, self.model.names, self.model.pt

    if len(results.xyxy[0]):
      for object in results.xyxy[0].tolist():
        if int(object[4] * 100) >= 30:
          cv2.rectangle(image, (int(object[0]), int(object[1])), (int(object[2]), int(object[3])),
                            (0, 255, 149), 2)
          cv2.putText(image, f'{names[int(object[5])]} - {int(object[4] * 100)}%',
                          (int(object[0]) + 20, int(object[1]) + 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                          (0, 255, 149),
                          2)
                
    return image

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
      rospy.loginfo(str(e))

    cv_image = self.find_boxes(cv_image)

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

if __name__ == '__main__':
    main()