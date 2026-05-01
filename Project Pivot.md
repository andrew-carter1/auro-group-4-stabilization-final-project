# Prject plan

## Komlan / Will: Drone control

* Spent a lot of time getting that installed
* Goal was to show a simulation of hardwawre before our hardware was available
* Decide: do we want to include that in our demo? if so are we able to launch it and show it working?

## Will: bgc connect

* bgc_connect: figured out how to decode the serial control method to the gimbal, good UART work we should document. 
* We were working with limited documentation (right?) 
    - only had access to the 32 bit uart API. Some features worked, some others didn't. 
* Script had some settings for speed callibration, IMU callibration... did we ever test that? could be useful to talk about
* Did you run the callibration? it's possible we never ran that and that's why we weren't seseing the best performance (using a default value instead). 
    - Also, the fact that we think that the speed values we were sending were oscillating with the gimbal's internal stabilization. 
    
    
## Adrian: Software stabilization

* Stabilizaton using only software, good work overall, lots to talk about, don't really need to add more to the demo
* Just need to figure out what folder/directory its located in. 


## Komlan: 

* Great work to show something physical in the demo if we can get the face detection working. 
* Wifi adapter, at office depot. not officially suppored but better than nothing? or do we just tell him. 


## Andrew: RS / yaw stabilizer

* Able to explain in the video
* Have pictures to show on the screen, showcasing how that works, explaining how we calculated the sensor readout times on various screens
* Move through kind of quick, as that was in semifinal demo
* Made use of gimbal IMU data. 
* I do have some video of that

Work today
* Test an improvement: Adding Kalman filter to average video calcualted movement + gimbal calculated movement. 
    - Could be applied to rolling shutter and yaw stabilizer
    - determine if it's worth making a neww node to output the change in yaw 
    - can explain how the gimbal's data is nnow obviousl very unreliable, so want to give that no gain. 

* Komlan filter: review this approach
    - good if its taking in a value of the location of the bounding box + the size of the bounding box, then applying constant velicoty/accl model to reduce noise (ie same as the homework). This is good because the siize and location currently have a lot of noise. Very similar
    to homework 4
    - 


## Changes i want to get to 

ok. a few changes that I want made in the @software/ros2_stabilization_ws/src/stabilization_pkg/ . First, uart_gimbal_servo, it's not a servo, it's for pitch and yaw control for the brushless motors in the gimbal. so if you can rename the word servo where it appears, that would be good (ie where it appears in the package), then rename the file to uart_pitch_roll.py (and rename the node where it exists and in the configuration).  








