	# leg1 front left
	# leg2 front right
	# leg3 back left
	# leg4 back right

	# x: positive forward
	# y: positive towards body center
	# z: positive up

	x_0 = 0.5
	x_stride = 1.0

	y_0 = 0.5 # offset towards body

	z_0 = -4.5
	z_lift = 1.0

	x_1 = x_0 + x_stride
	y_1 = -y_0
	z_1 = z_0 + z_lift

				#  move back left  |  move back right  | move front right  |  move front left  |
	x1_target = [x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_1, x_1, x_0]
	x2_target = [x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_0, x_1, x_1, x_1, x_1, x_1, x_1, x_0]
	x3_target = [x_0, x_0, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_0]
	x4_target = [x_0, x_0, x_0, x_0, x_0, x_0, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_1, x_0]

	y1_target = [y_1, y_1, y_1, y_1, y_0, y_0, y_0, y_0, y_0, y_0, y_0, y_1, y_1, y_1, y_1, y_1]
	y2_target = [y_0, y_0, y_0, y_0, y_1, y_1, y_1, y_1, y_1, y_1, y_1, y_0, y_0, y_0, y_0, y_0]
	y3_target = [y_1, y_1, y_1, y_1, y_0, y_0, y_0, y_0, y_0, y_0, y_0, y_1, y_1, y_1, y_1, y_1]
	y4_target = [y_0, y_0, y_0, y_0, y_1, y_1, y_1, y_1, y_1, y_1, y_1, y_0, y_0, y_0, y_0, y_0]

	z1_target = [z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_1, z_1, z_0, z_0]
	z2_target = [z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_1, z_1, z_0, z_0, z_0, z_0, z_0, z_0]
	z3_target = [z_0, z_1, z_1, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0]
	z4_target = [z_0, z_0, z_0, z_0, z_0, z_1, z_1, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0, z_0]


# initialize: x, y, and z positions for each foot & femur and tibia angles for each leg

res = 20.0
t = linspace(0,(len(x1_target)-1),(len(x1_target)-1)*res+1)

# zeros x, y, z, angs, angf, angt

x1 = zeros(len(t))
y1 = zeros(len(t))
z1 = zeros(len(t))
angs1 = zeros(len(t))
angf1 = zeros(len(t))
angt1 = zeros(len(t))

x2 = zeros(len(t))
y2 = zeros(len(t))
z2 = zeros(len(t))
angs2 = zeros(len(t))
angf2 = zeros(len(t))
angt2 = zeros(len(t))

x3 = zeros(len(t))
y3 = zeros(len(t))
z3 = zeros(len(t))
angs3 = zeros(len(t))
angf3 = zeros(len(t))
angt3 = zeros(len(t))

x4 = zeros(len(t))
y4 = zeros(len(t))
z4 = zeros(len(t))
angs4 = zeros(len(t))
angf4 = zeros(len(t))
angt4 = zeros(len(t))


# develop functions for foot positions for given gait

for i in range(0,len(t)-1):

	# find which section of the gait n to calculate

	n = int(floor(t[i]))

	x1[i] = x1_target[n] + (x1_target[n+1]-x1_target[n])/res*(i-n*res)
	x2[i] = x2_target[n] + (x2_target[n+1]-x2_target[n])/res*(i-n*res)
	x3[i] = x3_target[n] + (x3_target[n+1]-x3_target[n])/res*(i-n*res)
	x4[i] = x4_target[n] + (x4_target[n+1]-x4_target[n])/res*(i-n*res)

	y1[i] = y1_target[n] + (y1_target[n+1]-y1_target[n])/res*(i-n*res)
	y2[i] = y2_target[n] + (y2_target[n+1]-y2_target[n])/res*(i-n*res)
	y3[i] = y3_target[n] + (y3_target[n+1]-y3_target[n])/res*(i-n*res)
	y4[i] = y4_target[n] + (y4_target[n+1]-y4_target[n])/res*(i-n*res)

	z1[i] = z1_target[n] + (z1_target[n+1]-z1_target[n])/res*(i-n*res)
	z2[i] = z2_target[n] + (z2_target[n+1]-z2_target[n])/res*(i-n*res)
	z3[i] = z3_target[n] + (z3_target[n+1]-z3_target[n])/res*(i-n*res)
	z4[i] = z4_target[n] + (z4_target[n+1]-z4_target[n])/res*(i-n*res)