Title: simulation of soccer math with teams of two robots

team: Mahdi Shahrajabian, Amir Reza Sefidchian

Algorithms: for the forward player if direction = 0 then the robot goes forward otherwise rotates to follow the ball. for defender robot by default stays in the target and if strength is bigger than some threshold goes after the ball to take it away and then comes back to the target.

The P controller of the control system is slow, and there is a static error. With the PI controller the steady-state error disappeared, 
but the control system is slow. The PD compensation accelerates the system, but there is a static error. 
With the PID controller, the control system became faster and the steady-state error has been decreased to zero.

In order to better and simpler implementation the derivative part has been discretized with the backward difference approach
and the Integral part has been discretized with the forward diff
