class Dheading():
	def __init__(self,pos,heading):
		self.x = pos[1]  # x
		self.y = pos[0]  # y
		self.psi = heading  
        
	def DH(self):
		if self.psi < 90 and self.psi > 270:  
			if self.x <= 0.18 and self.x >= -0.18:
				psi_d = 0
			elif self.x > 0.18 and self.y >= -0.3:
				psi_d = 315
			elif self.x > 0.18 and self.y < -0.3:
				psi_d = 295
			elif self.x < -0.18 and self.y >= -0.3:
				psi_d = 45
			elif self.x < -0.18 and self.y < -0.3:
				psi_d = 65
		else:
			psi_d = 0
		return psi_d
