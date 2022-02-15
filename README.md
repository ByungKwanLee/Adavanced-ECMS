# ADVANCED-ECMS (Real-Time processing by ROS)
- **Author  : `Byung-Kwan Lee` (leebk@kaist.ac.kr)    M.S. Candidate**
- **Support : `Kyungjae Nam`  (tuta4048@kaist.ac.kr) M.S. Candidate**

## Requirements
- ROS kinetic version
- C++14

## Topic Message
- publisher   : **`/geometry_msgs/Twist.msg`**<br />
- subscriber  : **`std_msgs/Float32.msg, std_msgs/Int32.msg`**<br />

## Topic name of system variable
- accel          : **`/joy_teleop/cmd_vel`**<br /> (Twist.msg) --(ROS APP joystick)
- velocity       : **`/velocity`**<br />           (Float32.msg)
- soc            : **`/soc`**<br />                (Float32.msg)
- gear           : **`/gear`**<br />               (Int32.msg)
- mass           : **`/mass`**<br />               (Float32.msg)
- mode           : **`/mode`**<br />               (Int32.msg)
- lambda         : **`/lambda`**<br />             (Float32.msg)
- mu             : **`/mu`**<br />                 (Float32.msg)
- rho            : **`/rho`**<br />                (Float32.msg)

## Guide for Starting to Advanced_ECMS
- [1] ROS workspace and build work (catkin_make, source devel/setup.bash)
- [2] Access cfg folder, for editing ecms.yaml file
- [2-Example] ecms/ECMS_mode = "performance" or "realtime"
- [2-Example] ecms/method = "L" or "ADMM"
- [2-Example] ecms/iter is maximum iteration number
- [3] **`rosparam load ecms.yaml && cd ../../../ && rosrun ecms ecms`**, In terminal
- [4] If you want to change specification of Engine, Motor and, Battery, then you must change files in data folder, and edit in src/ICEMG.cpp
- [5] If you want to change driving cycle, then you must change files in data folder, and edit in src/performance.cpp
- [6] After performance ECMS_mode, driving cycle information csv files are saved in output folder. And, in iter folder inside output folder, driving cycle information when changing iteration from 1 to ecms/iter


## Method
- Iterative Lagrangian method
- Alternating Direction Method of Multipliers

## Reference
- [1] S. Boyd, “Distributed Optimization and Statistical Learning via the Alternating Direction Method of Multipliers,” Found. Trends® Mach. Learn., vol. 3, no. 1, pp. 1–122, 2010.
- [2] A. Sciarretta and L. Guzzella, “Control of Hybrid Electric Vehicles,” IEEE Control Syst. Mag., vol. 27, no. 2, pp. 60–70, 2007.


