Title: The robot rotates about the z-axis to adjust the desired heading angle.

This code generates the proper speed of robot motors for rotation of the robot to desired heading angle.

This code includes the implementation of the P, PI, PD and PID controllers for rotation of the robot to desired heading angle.

The P controller of the control system is slow, and there is a static error. With the PI controller the steady-state error disappeared, 
but the control system is slow. The PD compensation accelerates the system, but there is a static error.
With the PID controller, the control system became faster and the steady-state error has been decreased to zero.

In order to better and simpler implementation the derivative part has been discretized with the backward difference approach
and the Integral part has been discretized with the forward difference approach.

The motor response deadzone puts constraints on the control signal and inserts a delay in system response.

In addition of the above, saturation of control signal is also implemented in this code.
