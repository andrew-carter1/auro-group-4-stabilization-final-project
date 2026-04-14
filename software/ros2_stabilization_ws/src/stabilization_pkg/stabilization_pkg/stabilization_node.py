#! /usr/bin/env python3
"""
Class for detecting face and publish image with bounding box
"""

import rclpy 
import cv2 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError
import os


 
class MyFaceDetNode(Node):
  """ 
      Ros Node for 
      - subscribing to usb_cam image topic
      - detecting face, and 
      - publish image with bounding box 
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    super().__init__('face_detector') # your ros node

    # load cascade filter must be in ws (must be in workspace folder)
    CascadePath = os.path.abspath(os.getcwd())+'/src/my_face_detection/models/haarcascade.xml'
    self.faceCascade = cv2.CascadeClassifier(CascadePath)
    self.get_logger().info(f'{CascadePath}')
    self.mutex = Lock()

    self.bridge = CvBridge() # open-cv bridge
    self.topic = '/image_raw'
    self.subscription = self.create_subscription(Image, self.topic, self.imageCallBack, 1) # topic subscription
    self.subscription 

    # Bbox publisher and processed output image publisher
    self.publisher = self.create_publisher(Image, 'output/image', 1)



  def imageCallBack(self, r_im):
    """ 
    CallBack function to get the image through the 
        ros-opencv-bridge and start processing
    """
    try:
        self.original = self.bridge.imgmsg_to_cv2(r_im, "bgr8")
    except CvBridgeError as e:
        print(e)
    if self.original is None:
        print ('frame dropped, skipping detecting')
    else:
        self.ImageProcessor() 



  def ImageProcessor(self):
      """ 
      Process each frame
        - detect face and draw bounding boxes
        - publish out image
      """
      self.mutex.acquire()
      frame_out = self.detect_face(self.original)
      self.mutex.release()
      # transfer from OpenCV image to ROS data
      msg_frame = self.bridge.cv2_to_imgmsg(frame_out, encoding="bgr8")
      self.publisher.publish(msg_frame) # publish image
  


  def detect_face(self, image):
    """
    Detect face in given image (RGB/gray) and draw boudning boxes
    """
    if len(image.shape)==3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    faces = self.faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.CASCADE_SCALE_IMAGE
    )
    
    if faces is None:
        return image
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
    return image

  
def main(args=None):
  rclpy.init(args=None)
  face_detection = MyFaceDetNode()
  rclpy.spin(face_detection)


if __name__ == '__main__':
  main()
