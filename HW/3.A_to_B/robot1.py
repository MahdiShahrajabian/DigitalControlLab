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
                Ix = 0
                Iy = 0
                Ih = 0
                ex_prev = 0
                ey_prev = 0
                eh_prev = 0
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
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                print("pos = {}".format(robot_pos))
                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                

                # Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])

                # X position controller
                x_d = -0.3
                x = robot_pos[1]
                ex = x - x_d
                print("ex= {}".format(ex))
                
                PID_type_x = 'PID'
                print("PID_type_x = {}".format(PID_type_x))
                
                if PID_type_x == 'P':
                    # P controller Implementation 
                    Kpx = 1
                    Px = Kpx*ex
                    ux = Px
                elif PID_type_x == "PI":
                    # PI controller Implementation 
                    Kpx = 1
                    Kix = 0.4
                    Px = Kpx*ex
                    ux = Px + Ix
                elif PID_type_x == "PD":
                    # PD controller Implementation 
                    Kpx = 1   
                    Kdx = 0.1    # Kd/T
                    Px = Kpx*ex
                    Dx = Kdx * (ex-ex_prev)   # Backward Difference
                    ux = Px + Dx     
                else:
                    # PID controller Implementation 
                    Kpx = 1   
                    Kix = 0.4   # T*Ki
                    Kdx = 0.1     # Kd/T
                    Px = Kpx*ex
                    Dx = Kdx * (ex-ex_prev)   # Backward Difference
                    ux = Px + Ix + Dx
                
                # Saturation
                if ux>= 10:  
                    ux = 10
                if ux<= -10:
                    ux = -10
                    
                print("ux= {}".format(ux))
                
                # Y position controller
                y_d = 0.3
                y = robot_pos[0]
                ey = y - y_d
                print("ey= {}".format(ey))
                
                PID_type_y = 'PID'
                print("PID_type_y = {}".format(PID_type_y))
               
                if PID_type_y == 'P':
                    # P controller Implementation 
                    Kpy = 1
                    Py = Kpy*ey
                    uy = Py
                elif PID_type_y == "PI":
                    # PI controller Implementation 
                    Kpy = 1
                    Kiy = 0.4
                    Py = Kpy*ey
                    uy = Py + Iy
                elif PID_type_y == "PD":
                    # PD controller Implementation 
                    Kpy = 1   
                    Kdy = 0.2    # Kd/T
                    Py = Kpy*ey
                    Dy = Kdy * (ey-ey_prev)   # Backward Difference
                    uy = Py + Dy     
                else:
                    # PID controller Implementation 
                    Kpy = 1   
                    Kiy = 0.4   # T*Ki
                    Kdy = 0.1     # Kd/T
                    Py = Kpy*ey
                    Dy = Kdy * (ey-ey_prev)   # Backward Difference
                    uy = Py + Iy + Dy
                
                # Saturation
                if uy>= 10:
                    uy = 10
                if uy<=-10:
                    uy = -10
                    
                print("uy= {}".format(uy))
                
                
                # Heading controller
                psi_d = (180-(math.atan2((y_d-y0),(x_d-x0)))*180/3.1415) % 360
                print("psi_d= {}".format(psi_d))
                psi = self.get_compass_heading()*180/math.pi
                print("heading = {}".format(psi))
                eh = psi_d - psi
                
                PID_type_h = 'PI'
                print("PID_type_h = {}".format(PID_type_h))
               
                if PID_type_h == 'P':
                    # P controller Implementation 
                    Kph = 0.3
                    Ph = Kph*eh
                    w = Ph
                elif PID_type_h == "PI":
                    # PI controller Implementation 
                    Kph = 0.3
                    Kih = 0.05
                    Ph = Kph*eh
                    w = Ph + Ih
                elif PID_type_h == "PD":
                    # PD controller Implementation 
                    Kph = 0.3   
                    Kdh = 0.05    # Kd/T
                    Ph = Kph*eh
                    Dh = Kdh * (eh-eh_prev)   # Backward Difference
                    w = Ph + Dh     
                else:
                    # PID controller Implementation 
                    Kph = 0.3  
                    Kih = 0.05    # T*Ki
                    Kdh = 0.05     # Kd/T
                    Ph = Kph*eh
                    Dh = Kdh * (eh-eh_prev)   # Backward Difference
                    w = Ph + Ih + Dh
                    
                # Saturation
                if w>= 5: 
                    w = 5
                if w<=-5:
                    w = -5
                print("w= {}".format(w))
                
                V = math.sqrt(ux**2+uy**2)
                Vr = (2*ux-w*L)/(2*R)  # right motor speed
                Vl = (2*ux+w*L)/(2*R)  # left motor speed
                
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
                if PID_type_x == "PI":
                    Ix = Ix + Kix*ex   # Forward Difference
                elif PID_type_x == "PD":
                    ex_prev = ex
                elif PID_type_x == "PID":
                    Ix = Ix + Kix*ex
                    ex_prev = ex       
                else:
                    pass
                    
                if PID_type_y == "PI":
                    Iy = Iy + Kiy*ey   # Forward Difference
                elif PID_type_y == "PD":
                    ey_prev = ey
                elif PID_type_y == "PID":
                    Iy = Iy + Kiy*ey
                    ey_prev = ey       
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