import numpy as np
import cv2

def calculate_moving_average(curve, radius):
    # Calculate the moving average of a curve using a given radius
    window_size = 2 * radius + 1
    kernel = np.ones(window_size) / window_size
    curve_padded = np.pad(curve, (radius, radius), 'edge')
    smoothed_curve = np.convolve(curve_padded, kernel, mode='same')
    smoothed_curve = smoothed_curve[radius:-radius]
    return smoothed_curve

def smooth_trajectory(trajectory):
    # Smooth the trajectory using moving average on each dimension
    smoothed_trajectory = np.copy(trajectory)

    for i in range(3):
        smoothed_trajectory[:, i] = calculate_moving_average(
            trajectory[:, i],
            radius=SMOOTHING_RADIUS
        )

    return smoothed_trajectory

def fix_border(frame):
    # Fix the frame border by applying rotation and scaling transformation
    frame_shape = frame.shape
   
    matrix = cv2.getRotationMatrix2D(
        (frame_shape[1] / 2, frame_shape[0] / 2),
        0,
        1.04
    )

    frame = cv2.warpAffine(frame, matrix, (frame_shape[1], frame_shape[0]))
    return frame



SMOOTHING_RADIUS = 50

# use 0 to use webcam
cap = cv2.VideoCapture('input.mp4')

num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(num_frames, width, height, fps)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#don't define now
# out = cv2.VideoWriter('video_out.mp4', fourcc, fps, (2 * width, height))
#processing frames


#read the first frame
_, prev_frame = cap.read()

# convert to b&w
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)


# create transfomrations array to store info for each frame
transforms = np.zeros((num_frames - 1, 3), np.float32)



for i in range(num_frames - 2):

    # use the feature tracking functionality to detect point features in the
    # previous frame. 
    prev_points = cv2.goodFeaturesToTrack(
        prev_gray,
        maxCorners=200,
        qualityLevel=0.01,
        minDistance=30,
        blockSize=3
    )

    ## read the next (current) frame
    success, curr_frame = cap.read()

    if not success:
        break

    #convert to greyscale
    curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

    

    # https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html
    # use the two points to calculate the optical flow. This gives an estimate
    # of the motion of the frame, and returns the tracked points for the current
    # frame. 
    curr_points, status, err = cv2.calcOpticalFlowPyrLK(
        prev_gray,
        curr_gray,
        prev_points,
        None
    )

    # check to make sure the shape (num points? ) is the same
    assert prev_points.shape == curr_points.shape

    # only use the points that are valid in the new frame
    idx = np.where(status == 1)[0]
    prev_points = prev_points[idx]
    curr_points = curr_points[idx]

    # Estimate affine transformation between the points
    matrix, _ = cv2.estimateAffine2D(prev_points, curr_points)
    translation_x = matrix[0, 2]
    translation_y = matrix[1, 2]
    rotation_angle = np.arctan2(matrix[1, 0], matrix[0, 0])
    transforms[i] = [translation_x, translation_y, rotation_angle]
    prev_gray = curr_gray


# Calculate the trajectory by cumulatively summing the transformations
trajectory = np.cumsum(transforms, axis=0)

# Smooth the trajectory using moving average (3 frames?)
smoothed_trajectory = smooth_trajectory(trajectory)

# Calculate the difference between the smoothed and original trajectory
difference = smoothed_trajectory - trajectory

# Add the difference back to the original transformations to obtain smooth
# transformations
transforms_smooth = transforms + difference


# restart the capture frame
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

# Process each frame and stabilize the video
out = None  # Initialize as None

for i in range(num_frames - 2):
    success, frame = cap.read()
    
    if not success:
        break

    translation_x = transforms_smooth[i, 0]
    translation_y = transforms_smooth[i, 1]
    rotation_angle = transforms_smooth[i, 2]

    # Create the transformation matrix for stabilization
    transformation_matrix = np.zeros((2, 3), np.float32)
    transformation_matrix[0, 0] = np.cos(rotation_angle)
    transformation_matrix[0, 1] = -np.sin(rotation_angle)
    transformation_matrix[1, 0] = np.sin(rotation_angle)
    transformation_matrix[1, 1] = np.cos(rotation_angle)
    transformation_matrix[0, 2] = translation_x
    transformation_matrix[1, 2] = translation_y

    # Apply the transformation to stabilize the frame
    frame_stabilized = cv2.warpAffine(
        frame,
        transformation_matrix,
        (width, height)
    )

    # Fix the border of the stabilized frame
    frame_stabilized = fix_border(frame_stabilized)

    # Concatenate the original and stabilized frames side by side
    frame_out = cv2.hconcat([frame, frame_stabilized])

    if frame_out.shape[1] > 1920:
        frame_out = cv2.resize(
            frame_out,
            (frame_out.shape[1] // 2, frame_out.shape[0] // 2)
        )

    if out is None:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        h, w = frame_out.shape[:2]
        out = cv2.VideoWriter('video_out.mp4', fourcc, fps, (w, h))
        print(f"Writer initialized: {w}x{h}")

    cv2.imshow("Before and After", frame_out)
    cv2.waitKey(10)

    out.write(frame_out)