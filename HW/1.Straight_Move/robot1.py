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
                I_x = 0
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
                kix = 2
                ux = 20*ex + I_x
                
                if ux>= 10:  # Saturation
                    ux = 10
                if ux<=-10:
                    ux = -10
                    
                print("ux= {}".format(ux))
                
                # Set the speed to motorsd) 
                self.left_motor.setVelocity(ux) 
                self.right_motor.setVelocity(ux)  
                
                # Update State
                I_x = I_x + kix*ex   # FD
                ex_old = ex
 