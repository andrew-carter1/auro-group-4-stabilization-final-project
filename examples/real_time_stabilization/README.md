Real time stabilization example

https://www.makeuseof.com/opencv-real-time-video-stabilization-how-to-implement/


--to run:
cd ~/final_project 
colcon build --packages-select stabilization_pkg
source install/setup.bash
ros2 launch stabilization_pkg stabilization_launch.py