class PID():
    #first, we have to initialize the controller
	def __init__(self,SP,MV,KP,KI,KD,dt,Umax,Umin):
		self.kp = KP  # Proportional Gain
		self.ki = KI  # Integral Gain
		self.kd = KD  # Derivative Gain
		self.sp = SP  # Setpoint Value
		self.mv = MV  # Measured Variable
		self.dt = dt  # integral element
		self.error_last = 0  # Previous error
		self.integral_error = 0  # Integral of error
		self.saturation_max = Umax #max speed of the robot
		self.saturation_min = Umin #min speed of the robot
        
    #using the controller
	def compute(self):
		error =  self.mv - self.sp #compute the error
		derivative_error = (error - self.error_last) / self.dt #find the derivative of the error (how the error changes with time)  Backward Difference
		output = self.kp*error + self.ki*self.integral_error + self.kd*derivative_error 
		
		# Update State
		self.integral_error += error * self.dt #error build up over time
		self.error_last = error
		# Saturation
		if output > self.saturation_max:
			output = self.saturation_max
		elif output < self.saturation_min:
			output = self.saturation_min
		return output
