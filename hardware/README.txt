Must have PX4-Autopilot (in hardware/): git clone https://github.com/PX4/PX4-Autopilot.git --recursive
====================================================================
Demo Walkthrough: PX4 SITL + MAVROS Offboard Control in WSL
====================================================================
Goal: Launch a simulated drone, establish a ROS 2 bridge, bypass safety checks, execute a custom offboard hover script, and command a safe landing.

--------------------------------------------------------------------
Terminal 1: Launch Simulator & Bypass Safeties
--------------------------------------------------------------------
Purpose: Boot the 3D world and the drone's brain, then tell it to ignore the fact that we don't have a remote controller or a ground station connected.

1. Open a WSL terminal.
2. Navigate to your PX4 folder: 
   cd ~/auro-group-4-stabilization-final-project/hardware/PX4-Autopilot

3. Launch the simulation: 
   make px4_sitl gz_x500

4. Wait ~30 seconds for the drone to get a simulated GPS lock (ignore the initial yellow ekf2 warnings).

5. In this same terminal, press Enter to bring up the `pxh>` prompt.

6. Disable the Ground Control Station failsafe:
   param set NAV_DLL_ACT 0

7. (Optional) If it complains about a missing radio controller later, run:
   param set COM_RCL_EXCEPT 4

--------------------------------------------------------------------
Terminal 2: Establish the MAVROS Bridge
--------------------------------------------------------------------
Purpose: Start the node that translates ROS 2 topics/services into MAVLink commands the drone understands.

1. Open a second WSL terminal.
2. Source your base ROS 2 installation: 
   source /opt/ros/humble/setup.bash

3. Launch MAVROS (connecting to the SITL port):
   ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

--------------------------------------------------------------------
Gazebo Camera Setup (Interlude)
--------------------------------------------------------------------
Purpose: Ensure the audience can actually see the drone fly.

1. Go to the Gazebo window.
2. In the left panel (Entity Tree), find and right-click the `x500` model.
3. Select "Follow". Use the scroll wheel to zoom in so the drone stays framed.

--------------------------------------------------------------------
Terminal 3: Execute Custom Offboard Control
--------------------------------------------------------------------
Purpose: Run your custom Python node to send continuous 20Hz setpoints, switch to OFFBOARD mode, and arm the drone for a 2-meter hover.

1. Open a third WSL terminal.
2. Navigate to your custom ROS 2 workspace root:
   cd ~/auro-group-4-stabilization-final-project/hardware/ros2_ws

3. Source your built workspace:
   source install/setup.bash

4. Run the node:
   ros2 run drone_control offboard_node

Result: The drone will spool up its motors, take off, and hold a steady 2-meter hover.

--------------------------------------------------------------------
Terminal 4: Command the Landing
--------------------------------------------------------------------
Purpose: Safely end the flight by cutting off the Python script and issuing a manual land command.

1. CRITICAL FIRST STEP: Go back to Terminal 3 and press Ctrl+C to stop the Python node. (If you don't do this, it will fight your land command).

2. Open a fourth WSL terminal.
3. Source your base ROS 2 installation:
   source /opt/ros/humble/setup.bash

4. Send the AUTO.LAND service call:
   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'AUTO.LAND'}"

Result: The drone will descend, touch the ground, and automatically disarm.
