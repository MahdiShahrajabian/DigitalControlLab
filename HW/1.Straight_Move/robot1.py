# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math
#from turtle import position  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841
                global I
                I = 0
                global ex_prev
                ex_prev = 0
                global ux
                global Kix
                global ex
                
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841
                print("heading = {}".format(heading*180/3.1415))
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                print("pos = {}".format(robot_pos))
                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                

                # Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])

                x_d = -0.3
                x = robot_pos[1]
                ex = x - x_d
                print("ex= {}".format(ex))
                
                PID_type = 'PID'
                print("PID_type = {}".format(PID_type))
                
                if PID_type == 'P':
                    # P controller Implementation 
                    Kpx = 25
                    P = Kpx*ex
                    ux = P
                elif PID_type == "PI":
                    # PI controller Implementation 
                    Kpx = 22
                    Kix = 10
                    P = Kpx*ex
                    ux = P + I
                elif PID_type == "PD":
                    # PD controller Implementation 
                    Kpx = 23   
                    Kdx = 2     # Kd/T
                    P = Kpx*ex
                    D = Kdx * (ex-ex_prev)   # Backward Difference
                    ux = P + D     
                else:
                    # PID controller Implementation 
                    Kpx = 22   
                    Kix = 10   # T*Ki
                    Kdx = 1     # Kd/T
                    P = Kpx*ex
                    D = Kdx * (ex-ex_prev)   # Backward Difference
                    ux = P + I + D
                
                # Saturation
                if ux>= 10:  
                    ux = 10
                if ux<= -10:
                    ux = -10
                    
                print("ux= {}".format(ux))
                
                # Set the speed to motors 
                self.left_motor.setVelocity(ux) 
                self.right_motor.setVelocity(ux)  
                
                # Update State
                if PID_type == "PI":
                    I = I + Kix*ex   # Forward Difference
                elif PID_type == "PD":
                    ex_prev = ex
                elif PID_type == "PID":
                    I = I + Kix*ex
                    ex_prev = ex       
                else:
                    pass
