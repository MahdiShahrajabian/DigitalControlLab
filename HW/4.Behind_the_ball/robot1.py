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
                print("ed= {}".format(ed))
                
                PID_type_d = 'PID'
                print("PID_type_d = {}".format(PID_type_d))
                
                if PID_type_d == 'P':
                    # P controller Implementation 
                    Kpd = 200
                    Pd = Kpd*ed
                    ud = Pd
                elif PID_type_d == "PI":
                    # PI controller Implementation 
                    Kpd = 200
                    Kid = 50
                    Pd = Kpd*ed
                    ud = Pd + Id
                elif PID_type_d == "PD":
                    # PD controller Implementation 
                    Kpd = 200   
                    Kdd = 10    # Kd/T
                    Pd = Kpd*ed
                    Dd = Kdd * (ed-ed_prev)   # Backward Difference
                    ud = Pd + Dd     
                else:
                    # PID controller Implementation 
                    Kpd = 200   
                    Kid = 60   # T*Ki
                    Kdd = 10     # Kd/T
                    Pd = Kpd*ed
                    Dd = Kdd * (ed-ed_prev)   # Backward Difference
                    ud = Pd + Id + Dd
                
                # Saturation
                if ud>= 10:  
                    ud = 10
                if ud<= -10:
                    ud = -10
                    
                print("ud= {}".format(ud))
                
                # Heading controller
                psi_d = 0
                psi = self.get_compass_heading()*180/math.pi
                eh = psi_d - psi  # heading error

                PID_type_h = 'PI'
                print("PID_type_h = {}".format(PID_type_h))
               
                if PID_type_h == 'P':
                    # P controller Implementation 
                    Kph = 2
                    Ph = Kph*eh
                    w = Ph
                elif PID_type_h == "PI":
                    # PI controller Implementation 
                    Kph = 2
                    Kih = 0.3
                    Ph = Kph*eh
                    w = Ph + Ih
                elif PID_type_h == "PD":
                    # PD controller Implementation 
                    Kph = 2   
                    Kdh = 0.1   # Kd/T
                    Ph = Kph*eh
                    Dh = Kdh * (eh-eh_prev)   # Backward Difference
                    w = Ph + Dh     
                else:
                    # PID controller Implementation 
                    Kph = 2 
                    Kih = 0.3    # T*Ki
                    Kdh = 0.1     # Kd/T
                    Ph = Kph*eh
                    Dh = Kdh * (eh-eh_prev)   # Backward Difference
                    w = Ph + Ih + Dh
                    
                # Saturation
                if w>= 20: 
                    w = 20
                if w<=-20:
                    w = -20
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
                    V = ud
                    Vr = (2*V-w*L)/(2*R)
                    Vl = (2*V+w*L)/(2*R)
                   
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
                
                # Update State
                if PID_type_d == "PI":
                    Id = Id + Kid*ed   # Forward Difference
                elif PID_type_d == "PD":
                    ed_prev = ed
                elif PID_type_d == "PID":
                    Id = Id + Kid*ed   
                    ed_prev = ed       
                else:
                    pass
                    
                if PID_type_h == "PI":
                    Ih = Ih + Kih*eh   # Forward Difference
                elif PID_type_h == "PD":
                    eh_prev = eh
                elif PID_type_h == "PID":
                    Ih = Ih + Kih*eh
                    eh_prev = eh       
                else:
                    pass
