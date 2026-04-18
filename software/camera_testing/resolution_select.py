import sys
import cv2
import subprocess
from PyQt5 import QtWidgets, QtGui, QtCore

RESOLUTIONS = {
    "640x480": (640, 480),
    "800x600": (800, 600),
    "1280x720": (1280, 720),
    "1920x1080": (1920, 1080),
}

def list_devices():
    result = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True)
    lines = result.stdout.split("\n")

    devices = []
    current_name = None

    for line in lines:
        if "/dev/video" in line:
            dev = line.strip()
            devices.append((current_name, dev))
        elif line.strip():
            current_name = line.strip()

    return devices


def set_format(device, width, height):
    # Force MJPEG (helps avoid cropping/zoom issues)
    subprocess.run([
        "v4l2-ctl",
        f"--device={device}",
        f"--set-fmt-video=width={width},height={height},pixelformat=MJPG"
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class VideoThread(QtCore.QThread):
    frame_signal = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self):
        super().__init__()
        self.running = False
        self.device = "/dev/video0"

    def run(self):
        cap = cv2.VideoCapture(self.device)
        self.running = True

        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            img = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)

            self.frame_signal.emit(img)

        cap.release()

    def stop(self):
        self.running = False
        self.wait()


class App(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Linux Webcam (v4l2 + Qt)")

        self.label = QtWidgets.QLabel()
        self.label.setFixedSize(800, 600)
        self.label.setStyleSheet("background-color: black;")

        # Device dropdown
        self.device_combo = QtWidgets.QComboBox()
        self.devices = list_devices()
        for name, dev in self.devices:
            self.device_combo.addItem(f"{dev} ({name})", dev)

        # Resolution dropdown
        self.res_combo = QtWidgets.QComboBox()
        self.res_combo.addItems(RESOLUTIONS.keys())

        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.device_combo)
        layout.addWidget(self.res_combo)
        layout.addWidget(self.label)
        layout.addWidget(self.start_btn)
        layout.addWidget(self.stop_btn)
        self.setLayout(layout)

        self.thread = VideoThread()
        self.thread.frame_signal.connect(self.update_image)

        self.start_btn.clicked.connect(self.start_camera)
        self.stop_btn.clicked.connect(self.stop_camera)

    def start_camera(self):
        device = self.device_combo.currentData()
        res = self.res_combo.currentText()
        w, h = RESOLUTIONS[res]

        set_format(device, w, h)

        self.thread.device = device

        if not self.thread.isRunning():
            self.thread.start()

    def stop_camera(self):
        if self.thread.isRunning():
            self.thread.stop()

    def update_image(self, img):
        # Scale properly instead of cropping
        pix = QtGui.QPixmap.fromImage(img)
        pix = pix.scaled(self.label.size(), QtCore.Qt.KeepAspectRatio)
        self.label.setPixmap(pix)

    def closeEvent(self, event):
        self.stop_camera()
        event.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = App()
    win.show()
    sys.exit(app.exec_())
