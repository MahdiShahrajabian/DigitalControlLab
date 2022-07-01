# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math
#from turtle import position  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from PID import PID
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot2(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841
                t = 0
                L = 0.08
                R = 0.02
                Ix = 0
                Iy = 0
                Ih = 0
                ex_prev = 0
                ey_prev = 0
                eh_prev = 0
                robot_pos0 = self.get_gps_coordinates()
                x0 = robot_pos0[0]
                y0 = robot_pos0[1]
                
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    strength = ball_data["strength"]
                    print("strength2 = {}".format(strength))
                else:
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                print("pos2 = {}".format(robot_pos))
                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                
                direction = utils.get_direction(ball_data["direction"])
                t = t + 1*t
                x = robot_pos[0]
                y = robot_pos[1]
                psi = self.get_compass_heading()*180/math.pi
                print("heading2 = {}".format(psi))
                
                if (x > 0.1 or x < -0.1) and y < 0.6 and strength < 20:
                     x_d = 0
                     y_d = 0.7
                     ex = x_d - x
                     print("ex2= {}".format(ex))
                     ux = PID(x_d,x,1.2,50,0.001,0.01,10,-10).compute()
                     print("ux2= {}".format(ux))
                     ey = y_d - y
                     print("ey2= {}".format(ey))
                     uy = PID(y_d,y,1.2,50,0.001,0.01,10,-10).compute()
                     print("uy2= {}".format(uy))
                     psi_d = 0
                     print("psi_d2= {}".format(psi_d))                    
                     eh = psi_d - psi
                     w = PID(-psi_d,-psi,1.5,30,0,0.01,25,-25).compute()
                     print("w2= {}".format(w))
                     V = -math.sqrt(ux**2+uy**2)
                     Vr = (V-w*L)/(2*R)  # right motor speed
                     Vl = (V+w*L)/(2*R)  # left motor speed
                     if Vl>= 10: 
                         Vl = 10
                     if Vr<=-10:
                         Vr = -10
                     if Vr>= 10:
                         Vr = 10
                     if Vl<=-10:
                         Vl = -10
                elif strength > 20:
                     d_d = 0.001
                     d = 1/strength
                     ed = d - d_d
                     ud = PID(d_d,d,220,5000,0.1,0.01,10,-10).compute()
                     print("ud2= {}".format(ud))

                     if direction == 0:
                          Vl = ud
                          Vr = ud
                     elif direction != 0:   
                          Vl = direction * ud
                          Vr = direction * -ud 
                     if Vl>= 10: 
                         Vl = 10
                     if Vr<=-10:
                         Vr = -10
                     if Vr>= 10:
                         Vr = 10
                     if Vl<=-10:
                         Vl = -10               
                else:
                     Vl = 0
                     Vr = 0
                print("Vl2 = {}".format(Vl))
                print("Vr2 = {}".format(Vr))

                
                # Set the speed to motors 
                self.left_motor.setVelocity(Vl) 
                self.right_motor.setVelocity(Vr) 
                
