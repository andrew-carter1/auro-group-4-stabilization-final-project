import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, RegionOfInterest
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        model_path = os.path.expanduser('~/haarcascade_frontalface_default.xml')
        self.faceCascade = cv2.CascadeClassifier(model_path)
        self.bridge = CvBridge()

        # Subscriber: Get compressed stabilized frames
        self.subscription = self.create_subscription(
            CompressedImage, '/stabilized_frame/compressed', self.listener_callback, 10)

        # Publisher: Send image with faces drawn (compressed)
        self.publisher = self.create_publisher(CompressedImage, '/image_with_faces/compressed', 10)

        # Publisher: Send bounding box of largest face (RegionOfInterest)
        self.bbox_publisher = self.create_publisher(RegionOfInterest, '/face/bbox', 10)
        self.get_logger().info("Face Detection Node Started")

    def listener_callback(self, data):
        # Decode compressed image
        np_arr = np.frombuffer(data.data, np.uint8)
        imCV = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(imCV, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(gray, 1.1, 5)

        # Publish bounding box of largest face (if detected)
        if len(faces) > 0:
            # Find largest face by area
            largest_idx = max(range(len(faces)), key=lambda i: faces[i][2] * faces[i][3])
            x, y, w, h = faces[largest_idx]

            # Publish RegionOfInterest
            roi = RegionOfInterest()
            roi.x_offset = int(x)
            roi.y_offset = int(y)
            roi.width = int(w)
            roi.height = int(h)
            roi.do_rectify = False
            self.bbox_publisher.publish(roi)

        # Draw all detected faces on image
        for (x, y, w, h) in faces:
            cv2.rectangle(imCV, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Publish compressed image with drawn boxes
        _, buffer = cv2.imencode('.jpg', imCV)
        msg = CompressedImage()
        msg.header = data.header
        msg.format = "jpeg"
        msg.data = np.array(buffer).tobytes()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()