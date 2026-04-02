import numpy as np
import cv2
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

def fix_border(frame):
    frame_shape = frame.shape
    matrix = cv2.getRotationMatrix2D(
        (frame_shape[1] / 2, frame_shape[0] / 2),
        0,
        1.2
    )
    frame = cv2.warpAffine(frame, matrix, (frame_shape[1], frame_shape[0]))
    return frame


class StabilizationNode(Node):
    def __init__(self):
        super().__init__('stabilization_node')
        self.publisher = self.create_publisher(CompressedImage, '/stabilized_frame/compressed', 10)
        self.bridge = CvBridge()
        self.MIN_FEATURES = 5
        self.BUFFER_SIZE = 24
        self.trajectory = np.zeros(3)
        self.prev_transform = np.zeros(3)
        self.transform_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.frame_buffer = deque(maxlen=self.BUFFER_SIZE)
        self.frame_count = 0

        # ========== WEBCAM SETUP ==========
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Webcam: {self.width}x{self.height} @ {fps}fps")

        # Read first frame
        success, prev_frame = self.cap.read()
        if not success:
            self.get_logger().error("Failed to read from webcam!")
            return

        self.get_logger().info("First frame read successfully!")
        self.prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

        # Timer ~15fps
        self.timer = self.create_timer(0.066, self.process_frame)
        self.get_logger().info("Stabilization node started!")

    def process_frame(self):
        success, curr_frame = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to read frame")
            return

        self.frame_buffer.append(curr_frame.copy())
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

        # ========== FEATURE DETECTION AND TRACKING ==========
        prev_points = cv2.goodFeaturesToTrack(
            self.prev_gray,
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=30,
            blockSize=3
        )

        current_transform = self.prev_transform

        if prev_points is None or len(prev_points) < self.MIN_FEATURES:
            frame_out = cv2.hconcat([curr_frame, curr_frame])
            cv2.putText(frame_out, "Insufficient features detected",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            curr_points, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, curr_gray, prev_points, None
            )

            idx = np.where(status == 1)[0]
            prev_points = prev_points[idx]
            curr_points = curr_points[idx]

            # ========== MOTION ESTIMATION ==========
            if len(prev_points) >= 3:
                matrix, _ = cv2.estimateAffinePartial2D(prev_points, curr_points)
                if matrix is not None:
                    translation_x = matrix[0, 2]
                    translation_y = matrix[1, 2]
                    rotation_angle = np.arctan2(matrix[1, 0], matrix[0, 0])
                    current_transform = np.array([translation_x, translation_y, rotation_angle])

        self.transform_buffer.append(current_transform)
        self.trajectory += current_transform

        # ========== APPLY STABILIZATION ==========
        if len(self.transform_buffer) == self.BUFFER_SIZE:
            frame_to_stabilize = list(self.frame_buffer)[0]
            trajectory_array = np.array(self.transform_buffer)
            cumulative_trajectory = np.cumsum(trajectory_array, axis=0)
            smoothed_value = np.mean(cumulative_trajectory, axis=0)
            difference = smoothed_value - cumulative_trajectory[0]
            transform_smooth = trajectory_array[0] + difference

            translation_x = transform_smooth[0]
            translation_y = transform_smooth[1]
            rotation_angle = transform_smooth[2]

            transformation_matrix = np.zeros((2, 3), np.float32)
            transformation_matrix[0, 0] = np.cos(rotation_angle)
            transformation_matrix[0, 1] = -np.sin(rotation_angle)
            transformation_matrix[1, 0] = np.sin(rotation_angle)
            transformation_matrix[1, 1] = np.cos(rotation_angle)
            transformation_matrix[0, 2] = translation_x
            transformation_matrix[1, 2] = translation_y

            frame_stabilized = cv2.warpAffine(
                frame_to_stabilize, transformation_matrix, (self.width, self.height)
            )
            frame_stabilized = fix_border(frame_stabilized)
            frame_to_stabilize = fix_border(frame_to_stabilize)
            frame_out = cv2.hconcat([frame_to_stabilize, frame_stabilized])
            self.prev_transform = current_transform

            # Publish compressed stabilized frame
            _, buffer = cv2.imencode('.jpg', frame_stabilized)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = np.array(buffer).tobytes()
            self.publisher.publish(msg)
        else:
            frame_stabilized = curr_frame.copy()
            frame_out = cv2.hconcat([curr_frame, frame_stabilized])

        # ========== DISPLAY ==========
        if frame_out.shape[1] > 1920:
            frame_out = cv2.resize(
                frame_out, (frame_out.shape[1] // 2, frame_out.shape[0] // 2)
            )

        cv2.putText(frame_out, "Original", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame_out, "Stabilized", (self.width + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Real-time Stabilization", frame_out)
        cv2.waitKey(1)

        self.prev_gray = curr_gray
        self.frame_count += 1

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info(f"Processed {self.frame_count} frames")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StabilizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()