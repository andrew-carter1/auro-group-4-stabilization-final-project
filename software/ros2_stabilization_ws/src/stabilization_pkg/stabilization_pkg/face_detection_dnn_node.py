#!/usr/bin/env python3
"""
Face Detection DNN Node — Direct Camera Capture with SSD

Ports the HW1 C++ SSD face detector to Python. Opens USB camera directly
(V4L2 + MJPEG) and runs SSD face detection (res10_300x300_ssd).

Much more robust than Haar Cascade; works at various angles and distances.

Publishes:
  - /image_with_faces/compressed — annotated frames with face boxes + confidence
  - /face/bbox — RegionOfInterest of largest detected face
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, RegionOfInterest
import os


class FaceDetectionDNNNode(Node):
    def __init__(self):
        super().__init__('face_detection_dnn_node')

        # Load SSD model (Caffe-based, from HW1)
        proto_path, model_path = self._find_ssd_models()

        try:
            self.net = cv2.dnn.readNetFromCaffe(proto_path, model_path)
            self.get_logger().info(f"SSD model loaded: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load SSD model: {e}")
            raise

        # Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('capture_fps', 30.0)
        self.declare_parameter('confidence_threshold', 0.5)

        video_device = self.get_parameter('video_device').get_parameter_value().string_value
        self.width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('capture_fps').get_parameter_value().double_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # Open camera
        self.cap = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera: {video_device}")
            raise RuntimeError(f"Cannot open camera: {video_device}")

        # Configure camera
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f"Camera opened: {video_device} at {actual_w}×{actual_h} @ {actual_fps} fps"
        )

        # Publishers
        self.image_pub = self.create_publisher(CompressedImage, '/image_with_faces/compressed', 10)
        self.bbox_pub = self.create_publisher(RegionOfInterest, '/face/bbox', 10)

        # Timer for capture loop
        self.create_timer(1.0 / self.fps, self._capture_and_detect)

    def _find_ssd_models(self):
        """Find SSD model files from HW1 or local package."""
        # Try package-local first
        package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        proto_path = os.path.join(package_path, 'models', 'deploy.prototxt')
        model_path = os.path.join(package_path, 'models', 'res10_300x300_ssd_iter_140000.caffemodel')

        if os.path.exists(proto_path) and os.path.exists(model_path):
            return proto_path, model_path

        # Fall back to HW1
        hw1_proto = os.path.expanduser('~/Documents/Spring_2026/Auto_Robot_5934/Assignments/HW1/hw1_ws/src/hw1_package/models/deploy.prototxt')
        hw1_model = os.path.expanduser('~/Documents/Spring_2026/Auto_Robot_5934/Assignments/HW1/hw1_ws/src/hw1_package/models/res10_300x300_ssd_iter_140000.caffemodel')

        if os.path.exists(hw1_proto) and os.path.exists(hw1_model):
            self.get_logger().info(f"Using HW1 model files")
            return hw1_proto, hw1_model

        raise RuntimeError(
            f"SSD model files not found. Checked:\n"
            f"  {proto_path}\n"
            f"  {hw1_proto}"
        )

    def _capture_and_detect(self):
        """Capture frame, detect faces via SSD, publish results."""
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to read frame", throttle_duration_sec=2.0)
            return

        h, w = frame.shape[:2]

        # Prepare blob for DNN (SSD expects 300x300 input)
        blob = cv2.dnn.blobFromImage(
            frame,
            1.0,                              # scale
            (300, 300),                        # size
            [104.0, 177.0, 123.0],             # mean values (BGR)
            False,                             # swap RB
            False                              # crop
        )

        self.net.setInput(blob)
        detections = self.net.forward()

        # Extract faces with confidence > threshold
        faces = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.confidence_threshold:
                # Normalized coordinates
                x_min = detections[0, 0, i, 3]
                y_min = detections[0, 0, i, 4]
                x_max = detections[0, 0, i, 5]
                y_max = detections[0, 0, i, 6]

                # Scale to frame dimensions
                x1 = int(x_min * w)
                y1 = int(y_min * h)
                x2 = int(x_max * w)
                y2 = int(y_max * h)

                # Clamp to frame bounds
                x1 = max(0, min(x1, w - 1))
                y1 = max(0, min(y1, h - 1))
                x2 = max(x1 + 1, min(x2, w))
                y2 = max(y1 + 1, min(y2, h))

                box_w = x2 - x1
                box_h = y2 - y1
                faces.append((x1, y1, box_w, box_h, confidence))

        # Publish bounding box of largest face (if detected)
        if len(faces) > 0:
            largest = max(faces, key=lambda f: f[2] * f[3])
            x, y, box_w, box_h, conf = largest

            roi = RegionOfInterest()
            roi.x_offset = int(x)
            roi.y_offset = int(y)
            roi.width = int(box_w)
            roi.height = int(box_h)
            roi.do_rectify = False
            self.bbox_pub.publish(roi)

        # Draw all detected faces on image
        for x, y, box_w, box_h, conf in faces:
            cv2.rectangle(frame, (x, y), (x + box_w, y + box_h), (0, 255, 0), 2)
            cv2.putText(frame, f"{conf:.2f}", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish compressed image with drawn boxes
        _, buffer = cv2.imencode('.jpg', frame)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(buffer).tobytes()
        self.image_pub.publish(msg)

    def destroy_node(self):
        """Clean up camera on shutdown."""
        if self.cap:
            self.cap.release()
        self.get_logger().info("Camera released")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionDNNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
