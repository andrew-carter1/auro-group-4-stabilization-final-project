import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import serial
import time

SERIAL_PORT    = '/dev/ttyUSB0'
BAUD           = 115200
SEND_HZ        = 10      # how often to send commands to the ESP32

# Proportional gain: degrees of gimbal correction per pixel of face error.
# Increase to make tracking more aggressive; decrease if motors overshoot.
KP_YAW   = 0.05
KP_PITCH = 0.05
MAX_ANGLE = 30.0  # degrees, safety clamp


def send_control(ser: serial.Serial, pitch_deg: float, yaw_deg: float) -> None:
    msg = f'Y:{yaw_deg:.2f},P:{pitch_deg:.2f}\n'
    ser.write(msg.encode())


class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')

        self.subscription = self.create_subscription(
            Image, '/camera1/image_raw', self.callback, 3)
        self.pub = self.create_publisher(Image, '/out/image', 10)
        self.bridge = CvBridge()
        self.detector = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

        # Kalman filter — state: [cx, cy, vx, vy], measurement: [cx, cy]
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array(
            [[1, 0, 1, 0],
             [0, 1, 0, 1],
             [0, 0, 1, 0],
             [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0
        self.kf.errorCovPost        = np.eye(4, dtype=np.float32)
        self.kf_ready = False
        self._last_send = 0.0

        try:
            self.ser = serial.Serial(
                SERIAL_PORT, BAUD, timeout=0.05, dsrdtr=False, rtscts=False)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Gimbal serial open on {SERIAL_PORT}')
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f'No gimbal serial ({e}) — running vision only')

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w  = frame.shape[:2]
        cx_screen, cy_screen = w / 2.0, h / 2.0

        gray = cv2.equalizeHist(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        faces = self.detector.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=3, minSize=(80, 80))

        predicted = self.kf.predict()

        if len(faces) > 0:
            x, y, fw, fh = max(faces, key=lambda f: f[2] * f[3])
            cx, cy = float(x + fw / 2), float(y + fh / 2)

            if not self.kf_ready:
                self.kf.statePre  = np.array([[cx], [cy], [0.0], [0.0]], np.float32)
                self.kf.statePost = np.array([[cx], [cy], [0.0], [0.0]], np.float32)
                self.kf_ready = True

            self.kf.correct(np.array([[cx], [cy]], np.float32))
            tracked_cx, tracked_cy = cx, cy
            cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
        else:
            tracked_cx = float(predicted[0][0])
            tracked_cy = float(predicted[1][0])

        if self.kf_ready:
            error_x =  tracked_cx - cx_screen
            error_y =  tracked_cy - cy_screen

            yaw_deg   = float(np.clip(KP_YAW   * error_x, -MAX_ANGLE, MAX_ANGLE))
            pitch_deg = float(np.clip(-KP_PITCH * error_y, -MAX_ANGLE, MAX_ANGLE))

            now = time.monotonic()
            if self.ser and (now - self._last_send) >= 1.0 / SEND_HZ:
                send_control(self.ser, pitch_deg, yaw_deg)
                self._last_send = now
                self.get_logger().info(f'Sent: Y:{yaw_deg:.2f},P:{pitch_deg:.2f}')

            cv2.drawMarker(frame, (int(cx_screen), int(cy_screen)),
                           (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.circle(frame, (int(tracked_cx), int(tracked_cy)), 8, (0, 255, 255), 2)
            cv2.putText(frame, f'yaw={yaw_deg:+.1f}  pitch={pitch_deg:+.1f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Read and log any response from the ESP32
        if self.ser and self.ser.in_waiting:
            echo = self.ser.readline().decode(errors='ignore').strip()
            if echo:
                self.get_logger().info(f'ESP32: {echo}')

        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        cv2.imshow('Face Tracker', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
