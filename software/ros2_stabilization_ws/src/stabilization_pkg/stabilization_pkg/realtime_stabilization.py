import numpy as np
import cv2
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Vector3Stamped


def fix_border(frame):
    """Slightly zooms and crops to hide black edges from stabilization."""
    frame_shape = frame.shape
    matrix = cv2.getRotationMatrix2D(
        (frame_shape[1] / 2, frame_shape[0] / 2),
        0,
        1.1
    )
    return cv2.warpAffine(frame, matrix, (frame_shape[1], frame_shape[0]))


def sharpen(frame):
    """Unsharp mask to reduce blur and increase perceived sharpness."""
    blurred = cv2.GaussianBlur(frame, (0, 0), 3)
    return cv2.addWeighted(frame, 2.0, blurred, -1.0, 0)


class StabilizationNode(Node):
    def __init__(self):
        super().__init__('stabilization_node')

        # Publisher for stabilized frames
        self.publisher = self.create_publisher(
            CompressedImage, '/stabilized_frame/compressed', 10
        )

        # Publisher for raw frames (feeds yaw_stabilizer)
        self.raw_publisher = self.create_publisher(
            CompressedImage, '/raw_frame/compressed', 10
        )

        # Publisher for yaw angle (feeds yaw_stabilizer)
        self.gimbal_pub = self.create_publisher(
            Vector3Stamped, '/gimbal/angles', 10
        )

        # ========== WEBCAM SETUP ==========
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Webcam: {self.width}x{self.height} @ {fps}fps")

        # ========== EMA CONFIGURATION ==========
        self.ALPHA_XY  = 0.05
        self.ALPHA_YAW = 0.02
        self.MIN_FEATURES = 20

        # Trajectory tracking
        self.trajectory          = np.zeros(3)
        self.smoothed_trajectory = np.zeros(3)
        self.prev_transform      = np.zeros(3)

        self.frame_count = 0
        self.shake_history = deque(maxlen=100)

        # ========== INITIALIZATION ==========
        success, prev_frame = self.cap.read()
        if not success:
            self.get_logger().error("Failed to read from webcam!")
            return

        self.get_logger().info("First frame read successfully!")
        self.prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        self.get_logger().info("Starting real-time EMA stabilization with yaw + sharpening...")

        self.timer = self.create_timer(0.033, self.process_frame)

    def draw_shake_graph(self, frame_out, history, max_points=100):
        graph_h, graph_w = 80, frame_out.shape[1]
        graph = np.zeros((graph_h, graph_w, 3), dtype=np.uint8)

        for i in range(1, len(history)):
            x1 = int((i - 1) / max_points * graph_w)
            x2 = int(i / max_points * graph_w)

            y1 = int(graph_h / 2 - history[i-1][0])
            y2 = int(graph_h / 2 - history[i][0])
            cv2.line(graph, (x1, y1), (x2, y2), (255, 0, 0), 1)

            y1 = int(graph_h / 2 - history[i-1][1])
            y2 = int(graph_h / 2 - history[i][1])
            cv2.line(graph, (x1, y1), (x2, y2), (0, 255, 0), 1)

            y1 = int(graph_h / 2 - history[i-1][2] * 100)
            y2 = int(graph_h / 2 - history[i][2] * 100)
            cv2.line(graph, (x1, y1), (x2, y2), (0, 255, 255), 1)

        cv2.putText(graph, "X shake", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0,   0), 1)
        cv2.putText(graph, "Y shake", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,   255, 0), 1)
        cv2.putText(graph, "Yaw",     (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,   255, 255), 1)

        return np.vstack([frame_out, graph])

    def process_frame(self):
        success, curr_frame = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to read frame")
            return

        # ========== PUBLISH RAW FRAME IMMEDIATELY ==========
        _, raw_buffer = cv2.imencode('.jpg', curr_frame)
        raw_msg        = CompressedImage()
        raw_msg.format = "jpeg"
        raw_msg.data   = np.array(raw_buffer).tobytes()
        self.raw_publisher.publish(raw_msg)

        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

        # ========== FEATURE DETECTION AND TRACKING ==========
        prev_points = cv2.goodFeaturesToTrack(
            self.prev_gray,
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=30,
            blockSize=3
        )

        current_transform = self.prev_transform.copy()
        da = 0.0
        dx = 0.0
        dy = 0.0

        if prev_points is not None and len(prev_points) >= self.MIN_FEATURES:
            curr_points, status, err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, curr_gray, prev_points, None
            )

            idx = np.where(status == 1)[0]
            if len(idx) >= 3:
                prev_points = prev_points[idx]
                curr_points = curr_points[idx]

                matrix, _ = cv2.estimateAffinePartial2D(prev_points, curr_points)

                if matrix is not None:
                    dx = matrix[0, 2]
                    dy = matrix[1, 2]
                    da = np.arctan2(matrix[1, 0], matrix[0, 0])
                    current_transform = np.array([dx, dy, da])

        # ========== PUBLISH YAW TO /gimbal/angles ==========
        yaw_msg = Vector3Stamped()
        yaw_msg.header.stamp    = self.get_clock().now().to_msg()
        yaw_msg.header.frame_id = 'camera'
        yaw_msg.vector.x = 0.0
        yaw_msg.vector.y = 0.0
        yaw_msg.vector.z = np.degrees(da)
        self.gimbal_pub.publish(yaw_msg)

        # ========== EMA SMOOTHING ==========
        self.trajectory += current_transform

        if self.frame_count == 0:
            self.smoothed_trajectory = self.trajectory.copy()
        else:
            self.smoothed_trajectory[0] = (
                self.ALPHA_XY * self.trajectory[0] +
                (1 - self.ALPHA_XY) * self.smoothed_trajectory[0]
            )
            self.smoothed_trajectory[1] = (
                self.ALPHA_XY * self.trajectory[1] +
                (1 - self.ALPHA_XY) * self.smoothed_trajectory[1]
            )
            self.smoothed_trajectory[2] = (
                self.ALPHA_YAW * self.trajectory[2] +
                (1 - self.ALPHA_YAW) * self.smoothed_trajectory[2]
            )

        diff       = self.smoothed_trajectory - self.trajectory
        correction = current_transform + diff

        # ========== BUILD WARP MATRIX ==========
        tx, ty, ta = correction
        m       = np.zeros((2, 3), np.float32)
        m[0, 0] =  np.cos(ta)
        m[0, 1] = -np.sin(ta)
        m[1, 0] =  np.sin(ta)
        m[1, 1] =  np.cos(ta)
        m[0, 2] = tx
        m[1, 2] = ty

        # ========== APPLY STABILIZATION ==========
        frame_stabilized = cv2.warpAffine(
            curr_frame, m, (self.width, self.height), flags=cv2.INTER_LANCZOS4
        )
        frame_stabilized = fix_border(frame_stabilized)

        # ========== APPLY SHARPENING ==========
        frame_sharpened = sharpen(frame_stabilized)

        # ========== PUBLISH STABILIZED FRAME ==========
        _, buffer = cv2.imencode('.jpg', frame_sharpened)
        msg        = CompressedImage()
        msg.format = "jpeg"
        msg.data   = np.array(buffer).tobytes()
        self.publisher.publish(msg)

        # ========== DISPLAY ==========
        frame_out = cv2.hconcat([curr_frame, frame_stabilized, frame_sharpened])

        if frame_out.shape[1] > 1920:
            frame_out = cv2.resize(
                frame_out, (frame_out.shape[1] // 2, frame_out.shape[0] // 2)
            )

        cv2.putText(frame_out, "Original",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame_out, "Stabilized",
                    (self.width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame_out, "Sharpened",
                    (self.width * 2 + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        yaw_degrees = np.degrees(da)
        cv2.putText(frame_out, f"Yaw: {yaw_degrees:.2f} deg",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        self.shake_history.append(current_transform)
        frame_out = self.draw_shake_graph(frame_out, list(self.shake_history))

        cv2.imshow("Stabilization Pipeline", frame_out)
        cv2.waitKey(1)

        self.prev_gray      = curr_gray
        self.prev_transform = current_transform
        self.frame_count   += 1

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