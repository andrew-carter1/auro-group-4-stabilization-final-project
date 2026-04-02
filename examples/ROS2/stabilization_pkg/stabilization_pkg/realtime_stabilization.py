import numpy as np
import cv2
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

def fix_border(frame):
    """Slightly zooms and crops to hide black edges from stabilization."""
    frame_shape = frame.shape
    matrix = cv2.getRotationMatrix2D(
        (frame_shape[1] / 2, frame_shape[0] / 2),
        0,
        1.1
    )
    return cv2.warpAffine(frame, matrix, (frame_shape[1], frame_shape[0]))


class StabilizationNode(Node):
    def __init__(self):
        super().__init__('stabilization_node')

        # Publisher for stabilized frames
        self.publisher = self.create_publisher(CompressedImage, '/stabilized_frame/compressed', 10)

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

        # ========== EMA CONFIGURATION ==========
        # Higher ALPHA = follows movement faster (less stable)
        # Lower ALPHA = smoother (more stable, but might see more black borders)
        self.ALPHA = 0.05
        self.MIN_FEATURES = 20

        # Trajectory tracking
        self.trajectory = np.zeros(3)           # [x, y, angle] - cumulative raw motion
        self.smoothed_trajectory = np.zeros(3)  # EMA version of trajectory
        self.prev_transform = np.zeros(3)

        self.frame_count = 0

        # Shake graph history
        self.shake_history = deque(maxlen=100)

        # ========== INITIALIZATION ==========
        # Read the first frame
        success, prev_frame = self.cap.read()
        if not success:
            self.get_logger().error("Failed to read from webcam!")
            return

        self.get_logger().info("First frame read successfully!")

        # Convert to grayscale for feature tracking
        self.prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

        self.get_logger().info("Starting real-time EMA stabilization...")

        # Timer ~15fps
        self.timer = self.create_timer(0.066, self.process_frame)

    def draw_shake_graph(self, frame_out, history, max_points=100):
        graph_h, graph_w = 80, frame_out.shape[1]
        graph = np.zeros((graph_h, graph_w, 3), dtype=np.uint8)

        for i in range(1, len(history)):
            x1 = int((i - 1) / max_points * graph_w)
            x2 = int(i / max_points * graph_w)

            # X shake (blue)
            y1 = int(graph_h / 2 - history[i-1][0])
            y2 = int(graph_h / 2 - history[i][0])
            cv2.line(graph, (x1, y1), (x2, y2), (255, 0, 0), 1)

            # Y shake (green)
            y1 = int(graph_h / 2 - history[i-1][1])
            y2 = int(graph_h / 2 - history[i][1])
            cv2.line(graph, (x1, y1), (x2, y2), (0, 255, 0), 1)

        cv2.putText(graph, "X shake", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        cv2.putText(graph, "Y shake", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        return np.vstack([frame_out, graph])

    def process_frame(self):
        # Read current frame from webcam
        success, curr_frame = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to read frame")
            return

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

        if prev_points is not None and len(prev_points) >= self.MIN_FEATURES:
            # Optical Flow
            curr_points, status, err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, curr_gray, prev_points, None
            )

            idx = np.where(status == 1)[0]
            if len(idx) >= 3:
                prev_points = prev_points[idx]
                curr_points = curr_points[idx]

                # ========== MOTION ESTIMATION ==========
                matrix, _ = cv2.estimateAffinePartial2D(prev_points, curr_points)

                if matrix is not None:
                    dx = matrix[0, 2]
                    dy = matrix[1, 2]
                    da = np.arctan2(matrix[1, 0], matrix[0, 0])
                    current_transform = np.array([dx, dy, da])

        # ========== EMA SMOOTHING ==========
        # Accumulate the raw global position
        self.trajectory += current_transform

        if self.frame_count == 0:
            self.smoothed_trajectory = self.trajectory.copy()
        else:
            # Apply Exponential Moving Average: S = α*T + (1-α)*S_prev
            self.smoothed_trajectory = (
                (self.ALPHA * self.trajectory) +
                ((1 - self.ALPHA) * self.smoothed_trajectory)
            )

        # Calculate the correction (difference between smooth and raw)
        diff = self.smoothed_trajectory - self.trajectory

        # The actual transform to apply is the current step plus the corrective offset
        correction = current_transform + diff

        # ========== BUILD WARP MATRIX ==========
        tx, ty, ta = correction
        m = np.zeros((2, 3), np.float32)
        m[0, 0] = np.cos(ta)
        m[0, 1] = -np.sin(ta)
        m[1, 0] = np.sin(ta)
        m[1, 1] = np.cos(ta)
        m[0, 2] = tx
        m[1, 2] = ty

        # ========== APPLY STABILIZATION ==========
        frame_stabilized = cv2.warpAffine(
            curr_frame, m, (self.width, self.height), flags=cv2.INTER_LANCZOS4
        )
        frame_stabilized = fix_border(frame_stabilized)

        # ========== PUBLISH COMPRESSED STABILIZED FRAME ==========
        _, buffer = cv2.imencode('.jpg', frame_stabilized)
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(buffer).tobytes()
        self.publisher.publish(msg)

        # ========== DISPLAY ==========
        frame_out = cv2.hconcat([curr_frame, frame_stabilized])

        if frame_out.shape[1] > 1920:
            frame_out = cv2.resize(
                frame_out, (frame_out.shape[1] // 2, frame_out.shape[0] // 2)
            )

        cv2.putText(frame_out, "Original", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame_out, "Stabilized", (self.width + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Draw shake graph below video
        self.shake_history.append(current_transform)
        frame_out = self.draw_shake_graph(frame_out, list(self.shake_history))

        cv2.imshow("Real-time EMA Stabilization", frame_out)
        cv2.waitKey(1)

        # Prepare for next frame
        self.prev_gray = curr_gray
        self.prev_transform = current_transform
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