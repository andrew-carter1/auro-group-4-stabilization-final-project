import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Vector3Stamped
import cv2
import numpy as np
from collections import deque


class YawStabilizer(Node):
    def __init__(self):
        super().__init__('yaw_stabilizer')

        # subscribe to raw frame (before stabilization)
        self.sub = self.create_subscription(
            CompressedImage,
            '/raw_frame/compressed',
            self.callback,
            10
        )

        # subscribe to real gimbal yaw
        self.gimbal_sub = self.create_subscription(
            Vector3Stamped,
            '/gimbal/angles',
            self.gimbal_callback,
            10
        )

        # publish yaw corrected output
        self.pub = self.create_publisher(
            CompressedImage, '/yaw_stabilized/compressed', 10
        )

        self.current_yaw = 0.0
        self.yaw_history = deque(maxlen=30)

    def gimbal_callback(self, msg):
        # real yaw from gimbal hardware or optical flow
        self.current_yaw = msg.vector.z

    def callback(self, msg):
        # decode incoming frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        original = frame.copy()

        # smooth the yaw
        self.yaw_history.append(self.current_yaw)
        smoothed_yaw = np.mean(self.yaw_history)
        correction   = (smoothed_yaw - self.current_yaw) * 5

        # apply rotation correction
        h, w = frame.shape[:2]
        M     = cv2.getRotationMatrix2D((w // 2, h // 2), correction, 1.0)
        frame = cv2.warpAffine(frame, M, (w, h),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REFLECT)

        # overlays
        cv2.putText(frame, f"Gimbal Yaw: {self.current_yaw:.2f} deg",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Correction: {correction:.2f} deg",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # side by side display
        comparison = cv2.hconcat([original, frame])
        cv2.putText(comparison, "Raw",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(comparison, "Yaw Corrected",
                    (w + 10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Yaw Stabilizer", comparison)
        cv2.waitKey(1)

        # publish corrected frame
        _, buffer = cv2.imencode('.jpg', frame)
        out_msg        = CompressedImage()
        out_msg.format = "jpeg"
        out_msg.data   = np.array(buffer).tobytes()
        self.pub.publish(out_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YawStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()