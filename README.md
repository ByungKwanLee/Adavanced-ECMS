# ADVANCED_ECMS (Real-Time processing by ROS)
- **Author  : `ByungKwan Lee` (leebk@kaist.ac.kr)    M.S. Candidate**
- **Support : `KyungJae Nam`  (tuta4048@kaist.ac.kr) M.S. Candidate**
- **Advisor : `DongSuk Kum`   (dskum@kaist.ac.kr)    Professor**

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

## Method
- Iterative Lagrangian method
- Alternating Direction Method of Multipliers

## Reference
- [1] S. Boyd, “Distributed Optimization and Statistical Learning via the Alternating Direction Method of Multipliers,” Found. Trends® Mach. Learn., vol. 3, no. 1, pp. 1–122, 2010.
- [2] A. Sciarretta and L. Guzzella, “Control of Hybrid Electric Vehicles,” IEEE Control Syst. Mag., vol. 27, no. 2, pp. 60–70, 2007.

