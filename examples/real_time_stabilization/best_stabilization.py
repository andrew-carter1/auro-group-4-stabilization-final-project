import numpy as np
import cv2
from collections import deque

# ========== CONFIGURATION ==========
BUFFER_SIZE = 21 # Must be odd. ~0.8s delay at 30fps.
SIGMA = 10       # Strength of smoothing.
CROP_FACTOR = 1.2 # Zoom to hide edges.
# ===================================

cap = cv2.VideoCapture(0)
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

def get_gaussian_kernel(size, sigma):
    """Creates a 1D Gaussian kernel for temporal smoothing."""
    kernel = np.fromfunction(
        lambda x: (1/(np.sqrt(2*np.pi)*sigma)) * np.exp(-((x-(size-1)/2)**2)/(2*sigma**2)),
        (size,)
    )
    return kernel / np.sum(kernel)

# Initialize
frame_buffer = deque(maxlen=BUFFER_SIZE)
transform_buffer = deque(maxlen=BUFFER_SIZE)
kernel = get_gaussian_kernel(BUFFER_SIZE, SIGMA)

success, prev_frame = cap.read()
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

print("Buffering for Gaussian stabilization... Press 'q' to quit.")

while True:
    success, curr_frame = cap.read()
    if not success: break
    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

    # 1. Estimate Motion (Current vs Previous)
    p_prev = cv2.goodFeaturesToTrack(prev_gray, 200, 0.01, 30)
    step_transform = np.zeros(3)

    if p_prev is not None:
        p_curr, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, p_prev, None)
        idx = np.where(status == 1)[0]
        if len(idx) >= 3:
            m_affine, _ = cv2.estimateAffinePartial2D(p_prev[idx], p_curr[idx])
            if m_affine is not None:
                dx = m_affine[0, 2]
                dy = m_affine[1, 2]
                da = np.arctan2(m_affine[1, 0], m_affine[0, 0])
                step_transform = np.array([dx, dy, da])

    # 2. Update Window
    frame_buffer.append(curr_frame)
    transform_buffer.append(step_transform)
    
    if len(frame_buffer) == BUFFER_SIZE:
        # 3. Calculate Gaussian Weighted Path
        steps = np.array(transform_buffer)
        local_trajectory = np.cumsum(steps, axis=0)

        smoothed_point = np.zeros(3)
        for i in range(3):
            smoothed_point[i] = np.sum(local_trajectory[:, i] * kernel)

        # Center frame is the one we are actually processing
        center_idx = BUFFER_SIZE // 2
        original_point = local_trajectory[center_idx]
        diff = smoothed_point - original_point
        
        # Warp Parameters
        tx = steps[center_idx, 0] + diff[0]
        ty = steps[center_idx, 1] + diff[1]
        ta = steps[center_idx, 2] + diff[2]

        # 4. Transform with High-Quality Interpolation
        M = cv2.getRotationMatrix2D((width/2, height/2), np.degrees(ta), CROP_FACTOR)
        M[0, 2] += tx
        M[1, 2] += ty

        target_frame = frame_buffer[center_idx]
        frame_stabilized = cv2.warpAffine(target_frame, M, (width, height), flags=cv2.INTER_LANCZOS4)
        
        # 5. Side-by-Side Construction
        # We also zoom the 'Original' slightly so the scale matches for comparison
        M_orig = cv2.getRotationMatrix2D((width/2, height/2), 0, CROP_FACTOR)
        frame_original_zoomed = cv2.warpAffine(target_frame, M_orig, (width, height))

        frame_out = cv2.hconcat([frame_original_zoomed, frame_stabilized])

        # Add Labels
        cv2.putText(frame_out, "Original (Zoomed)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame_out, "Gaussian Stabilized", (width + 20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Scale down for display ONLY if it exceeds screen width (e.g., 2x1080p = 3840px)
        display_frame = frame_out
        if frame_out.shape[1] > 1600: 
            scale = 1600 / frame_out.shape[1]
            display_frame = cv2.resize(frame_out, (0,0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        cv2.imshow("Stabilization Comparison", display_frame)

    prev_gray = curr_gray
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()