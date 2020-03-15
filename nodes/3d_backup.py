	if (data.pose.position.z>0)
		Adxz = arctan(-data.pose.position.y/-data.pose.position.z)
	elif (data.pose.position.y==0)
		Adxz = pi
	else:
		Adxz = pi + arctan(-data.pose.position.y/-data.pose.position.z)

	dxz = sqrt(-data.pose.position.z**2 + -data.pose.position.y**2)
	As = Adxz - arccos(self.length_s/dxz)


	if (data.pose.position.x>0):
		Ad = arctan((-data.pose.position.y+self.length_s*sin(As))/-data.pose.position.x)
	else:
		Ad = pi + arctan((-data.pose.position.y+self.length_s*sin(As))/-data.pose.position.x)

	d = sqrt(-data.pose.position.x**2 + (-data.pose.position.y+self.length_s*sin(As))**2)
	Af = Ad - arccos((self.length_f**2 + d**2 - self.length_t**2)/(2*self.length_f*d))
	At = pi - arccos((self.length_f**2 + self.length_t**2 - d**2)/(2*self.length_f*self.length_t))