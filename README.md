# ros2_theta_cam
This package can publish images obtained from RICOH's THETA omnidirectional camera.

# Requirements
- sensor_msgs
- cv_bridge
- [nickel110/libuvc](https://github.com/nickel110/libuvc)
- [gstthetauvc](https://github.com/nickel110/gstthetauvc)

The Dockerfile can be used to prepare a development environment.

# Parameters
- mode   : THETA mode to decide image resolution: 2K or 4K
- serial : The serial number of the THETA to use.
         Useful if multiple THETAs are connected to the system.

# How to use
1. Check that your ThetaV is Live-Streaming-Mode, and run!

   ```
   ros2 run ros2_theta_cam theta_cam
   ```

2. If you want to check image outputs, launch rviz2 with another terminal.

   ```
   rviz2
   ```

  And "Add" -> "By topic" -> "/image_topic -> Image"
