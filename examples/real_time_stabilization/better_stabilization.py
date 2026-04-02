import numpy as np
import cv2

def fix_border(frame):
    """Slightly zooms and crops to hide black edges from stabilization."""
    frame_shape = frame.shape
    # zooms in by factor of 1.1
    matrix = cv2.getRotationMatrix2D(
        (frame_shape[1] / 2, frame_shape[0] / 2),
        0,
        1.1
    )
    return cv2.warpAffine(frame, matrix, (frame_shape[1], frame_shape[0]))

# ========== CONFIGURATION ==========
cap = cv2.VideoCapture(0)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

# Higher ALPHA = follows movement faster (less stable)
# Lower ALPHA = smoother (more stable, but might see more black borders)
ALPHA = 0.05
MIN_FEATURES = 20

# Trajectory tracking
trajectory = np.zeros(3)           # [x, y, angle] - cumulative raw motion
smoothed_trajectory = np.zeros(3)  # EMA version of trajectory
prev_transform = np.zeros(3)

# Initialize first frame
success, prev_frame = cap.read()
if not success:
    exit("Webcam not found.")

prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

print("Starting real-time stabilization... Press 'q' to quit")

frame_count = 0
while True:
    success, curr_frame = cap.read()
    if not success:
        break

    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
    
    # 1. Feature Detection
    prev_points = cv2.goodFeaturesToTrack(
        prev_gray, 
        maxCorners=200, 
        qualityLevel=0.01, 
        minDistance=30, 
        blockSize=3
    )
    
    current_transform = prev_transform.copy()

    if prev_points is not None and len(prev_points) >= MIN_FEATURES:
        # 2. Optical Flow
        curr_points, status, err = cv2.calcOpticalFlowPyrLK(prev_gray, 
                                                            curr_gray, 
                                                            prev_points, 
                                                            None)
        
        idx = np.where(status == 1)[0]
        if len(idx) >= 3:
            prev_points = prev_points[idx]
            curr_points = curr_points[idx]
            
            # 3. Motion Estimation
            matrix, _ = cv2.estimateAffinePartial2D(prev_points, curr_points)
            
            if matrix is not None:
                dx = matrix[0, 2]
                dy = matrix[1, 2]
                da = np.arctan2(matrix[1, 0], matrix[0, 0])
                current_transform = np.array([dx, dy, da])

    # 4. EMA Smoothing Logic
    # Accumulate the raw global position
    trajectory += current_transform

    if frame_count == 0:
        smoothed_trajectory = trajectory.copy()
    else:
        # Apply Exponential Moving Average: S = α*T + (1-α)*S_prev
        smoothed_trajectory = (ALPHA * trajectory) + ((1 - ALPHA) * smoothed_trajectory)

    # Calculate the correction (difference between smooth and raw)
    diff = smoothed_trajectory - trajectory
    
    # The actual transform to apply is the current step plus the corrective offset
    correction = current_transform + diff
    
    # 5. Build Warp Matrix
    tx, ty, ta = correction
    m = np.zeros((2, 3), np.float32)
    m[0, 0] = np.cos(ta)
    m[0, 1] = -np.sin(ta)
    m[1, 0] = np.sin(ta)
    m[1, 1] = np.cos(ta)
    m[0, 2] = tx
    m[1, 2] = ty

    # 6. Apply & Display
    frame_stabilized = cv2.warpAffine(curr_frame, m, (width, height), flags=cv2.INTER_LANCZOS4)
    frame_stabilized = fix_border(frame_stabilized)

    # UI Feedback
    frame_out = cv2.hconcat([curr_frame, frame_stabilized])
    cv2.putText(frame_out, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(frame_out, "Stabilized", (width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("Real-time EMA Stabilization", frame_out)
    
    # Prepare for next frame
    prev_gray = curr_gray
    prev_transform = current_transform
    frame_count += 1
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()