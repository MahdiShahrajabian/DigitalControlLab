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
                
                L = 0.08
                R = 0.02
                Ih = 0
                eh_prev = 0
                
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
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                print("pos = {}".format(robot_pos))
                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                

                # Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])
                
                # Heading controller
                psi_d = 80
                print("psi_d= {}".format(psi_d))
                psi = self.get_compass_heading()*180/math.pi
                print("heading = {}".format(psi))
                eh = psi_d - psi
                
                PID_type_h = 'PID'
                print("PID_type_h = {}".format(PID_type_h))
               
                if PID_type_h == 'P':
                    # P controller Implementation 
                    Kph = 0.1
                    Ph = Kph*eh
                    w = Ph
                elif PID_type_h == "PI":
                    # PI controller Implementation 
                    Kph = 0.1
                    Kih = 0.05
                    Ph = Kph*eh
                    w = Ph + Ih
                elif PID_type_h == "PD":
                    # PD controller Implementation 
                    Kph = 0.1  
                    Kdh = 0.05    # Kd/T
                    Ph = Kph*eh
                    Dh = Kdh * (eh-eh_prev)   # Backward Difference
                    w = Ph + Dh     
                else:
                    # PID controller Implementation 
                    Kph = 0.1 
                    Kih = 0.05    # T*Ki
                    Kdh = 0.05     # Kd/T
                    Ph = Kph*eh
                    Dh = Kdh * (eh-eh_prev)   # Backward Difference
                    w = Ph + Ih + Dh
                    
                # Saturation
                if w>= 4: 
                    w = 4
                if w<=-4:
                    w = -4
                print("w= {}".format(w))
                V = 0
                Vr = (2*V-w*L)/(2*R)  # right motor speed
                Vl = (2*V+w*L)/(2*R)  # left motor speed
                
                # Saturation
                if Vl>= 10: 
                    Vl = 10
                if Vr<=-10:
                    Vr = -10
                if Vr>= 10:
                    Vr = 10
                if Vl<=-10:
                    Vl = -10
                    
                print("Vl {}".format(Vl))
                print("Vr= {}".format(Vr))

                
                # Set the speed to motors 
                self.left_motor.setVelocity(Vl) 
                self.right_motor.setVelocity(Vr) 
                
                # Update State
                
                if PID_type_h == "PI":
                    Ih = Ih + Kih*eh   # Forward Difference
                elif PID_type_h == "PD":
                    eh_prev = eh
                elif PID_type_h == "PID":
                    Ih = Ih + Kih*eh
                    eh_prev = eh       
                else:
                    pass

                
