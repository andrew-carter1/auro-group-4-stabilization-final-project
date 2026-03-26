import rclpy # Python library for ROS2
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Convert between ROS and OpenCV Images
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, '/camera1/image_raw', self.listener_callback, 3)

        self.subscription
        
        self.ImOut= self.create_publisher(Image, '/out/image', 10)
        self.bridge = CvBridge()

        self.faceCascade = cv2.CascadeClassifier("./haarcascade_frontalface_default.xml")

    def listener_callback(self, data):
        imCV = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        gray = cv2.cvtColor(imCV, cv2.COLOR_BGR2GRAY)
        
        faces = self.faceCascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )
        
        for (x, y, w, h) in faces:
            cv2.rectangle(imCV, (x, y), (x+w, y+h), (0, 255, 0), 2)

        processed_msg = self.bridge.cv2_to_imgmsg(imCV, "bgr8")
        self.ImOut.publish(processed_msg)

        cv2.imshow("Face Detection Window", imCV)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()