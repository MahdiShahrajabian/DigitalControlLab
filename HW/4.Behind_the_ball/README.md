
Title: The robot tracks the ball and is placed behind it.

This code generates the proper speed of robot motors for tracking the ball.

This code includes the implementation of the P, PI, PD and PID controllers for movement of the robot.

The P controller of the control system is slow, and there is a static error. With the PI controller the steady-state error disappeared, 
but the control system is slow. The PD compensation accelerates the system, but there is a static error. 
With the PID controller, the control system became faster and the steady-state error has been decreased to zero.

In order to better and simpler implementation the derivative part has been discretized with the backward difference approach
and the Integral part has been discretized with the forward difference approach.

The motor response deadzone puts constraints on the control signal and inserts a delay in system response.

In addition of the above, saturation of control signal is also implemented in this code.
