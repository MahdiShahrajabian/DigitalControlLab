 
# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math
#from turtle import position  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from PID import PID
from Dheading import Dheading
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841
                
                L = 0.08
                R = 0.02
                Id = 0
                Ih = 0
                d_prev = 0
                ed_prev = 0
                eh_prev = 0
                
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    dir_vec = ball_data["direction"]
                    strength = ball_data["strength"]
                    print("dir= {}".format(dir_vec))
                    print("strength= {}".format(strength))

                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841
                print("heading = {}".format(heading*180/math.pi))
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                print("pos = {}".format(robot_pos))
                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                # print("sonar = {}".format(sonar_values))
				
                # Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])
                print("direction= {}".format(direction))
                
                # Distance PID controller    
                d_d = 0.001
                d = 1/strength
                ed = d - d_d
                ud = PID(d_d,d,200,5000,0.1,0.01,10,-10).compute()
		
                print("ud= {}".format(ud))
                
                # Heading controller
                
                psi = self.get_compass_heading()*180/math.pi
                psi_d = Dheading(robot_pos,psi).DH()
                eh = psi_d - psi  # heading error
               
                w = PID(d_d,d,3,30,0,0.01,30,-30).compute()

                print("w= {}".format(w))
                
			
                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise
                if direction == 0 and ed > 0.02:
                    left_speed = ud
                    right_speed = ud
                elif direction != 0 and ed > 0.02:
                
                    left_speed = direction * ud
                    right_speed = direction * -ud       
                        
                else:
                    V = 0.7*ud
                    Vr = (V-w*L)/(2*R)
                    Vl = (V+w*L)/(2*R)
                   
                    if Vl>= 10: # Saturation
                        Vl = 10
                    if Vr<=-10:
                        Vr = -10
                    if Vr>= 10:
                        Vr = 10
                    if Vl<=-10:
                        Vl = -10
                        
                    print("Vl = {}".format(Vl))
                    print("Vr = {}".format(Vr))
                    
                    left_speed = Vl
                    right_speed = Vr


                # Set the speed to motors 
                self.left_motor.setVelocity(left_speed) 
                self.right_motor.setVelocity(right_speed) 
                
