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
                Ix = 0
                Ih = 0
                robot_pos0 = self.get_gps_coordinates()
                x0 = robot_pos0[1]
                y0 = robot_pos0[0]
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


                x_d = -0.2
                y_d = 0.2
                x = robot_pos[1]
                ex = x - x_d
                print("ex= {}".format(ex))
                kix = 0.1
                Ix = Ix + kix*ex
                ux = 0.6*ex + Ix
                if ux>= 10:  # Saturation
                    ux = 10
                if ux<=-10:
                    ux = -10
                print("ux= {}".format(ux)) 
                
                psi_d = 50
                print("psi_d= {}".format(psi_d))
                psi = self.get_compass_heading()*180/3.1415 
                eh = psi_d - psi
                L = 0.08
                R = 0.02
                kh = 0.3
                khi = 0.05
                Ih = Ih + khi*eh
                w = kh*eh + Ih
                if w>= 5: # Saturation
                    w = 5
                if w<=-5:
                    w = -5
                print("w= {}".format(w))
                Vr = (2*ux-w*L)/(2*R)
                Vl = (2*ux+w*L)/(2*R)
                
                if Vl>= 10: # Saturation
                    Vl = 10
                if Vr<=-10:
                    Vr = -10
                if Vr>= 10:
                    Vr = 10
                if Vl<=-10:
                    Vl = -10
                print("Vl {}".format(Vl))
                print("Vr= {}".format(Vr))
                self.left_motor.setVelocity(Vl) 
                self.right_motor.setVelocity(Vr) 
                
                

                
