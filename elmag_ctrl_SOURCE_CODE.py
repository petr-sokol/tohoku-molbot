#second fresh start for Dobot Magnet
import DobotDllType as dType
import numpy as np
import hm310p
import cv2
import time

#for testing purposes, to run the program without the robotic arm 
#to test different parts of the code
connected = 0
if connected:
	api = dType.load()
	dType.ConnectDobot(api, "COM3", 115200)
	dType.SetPTPJumpParams(api, 100, 0, 0)
else: print("> Dobot arm not connected")


#checks for connection with power supply
try: hm = hm310p.HM310P()
except: print("> Power supply not connected.")

global li, vc, lines
li, vc = [], []
lines = []

S = [ 0, -240] #coordinates of the origin in workspace, new [0, 0]
R = 47 #to see the magnet positions, adjust R to 47, but it will collide with the maze
u = -3/4*np.pi #initial orientation of the magnet





## INPUT
#module from OpenCV to register user input with mouse clicks
#the coordinates are saved in a vector li
#lines vector is for drawing lines in the UI later on
f = cv2.FONT_HERSHEY_SIMPLEX
def klik(event, x, y, flags, params):
	if event == cv2.EVENT_LBUTTONDOWN:
		li.append([x,y])
		lines.append((x,y))
		# print("L", x, y)
		cv2.putText(img, '+', (x-13, y+9), f, 1, (75, 220, 235), 3)
		cv2.imshow('image', img)

img = cv2.imread("maze 05 grid.jpg", 1) #projection of the maze image on the screen
cv2.imshow("image", img)
cv2.setMouseCallback("image", klik)
cv2.waitKey(0)





## IMAGE TO COORDS
#mouse click coordinates need to be recalculated to the workspace coordinates
dim = img.shape
TopLeft = [-60,-180]
x0 = TopLeft[0]
y0 = TopLeft[1]
sx = 120 #size of grid in cm
sy = 120

for i in li: #recalculation of coordinate system
	i[0] =  i[0]*sx/dim[0]+x0
	i[1] = -i[1]*sy/dim[1]+y0





## MAGNET POSITION
#algorithm to convert the coordinates of points to correct magnet position and orientation
i, n = 0, len(li)-1
while i < n:
	x, xx = li[i][0], li[i+1][0] #starting points and ending points for each line segment
	y, yy = li[i][1], li[i+1][1]

	#v ~ vector
	#n ~ vector norm (length)
	#u ~ unit vector

	#PT~ (initial)Position-Target(position)
	#PT~ Position-Center(Střed in czech)


	vPT = [xx - x, yy - y] 
	nPT = np.linalg.norm(vPT)
	uPT = vPT/nPT

	eta = (np.arctan2(-vPT[1], -vPT[0])-u)*180/np.pi #angle between the line segment and initial robot arm orientation

	if eta > 60: eta -= 360 #making sure eta is from <0°, 360°>
	if eta < -240 or eta > 60: # new point addition for otherwise inaccessible robot arm orientations
		li.insert(i+1, [xx, y])

		xx = round((xx-x0)*dim[0]/sx) #render
		y  =-round((y-y0)*dim[1]/sy)  #projecting additional created points on the UI (in red)
		cv2.putText(img, '+', (xx-13, y+9), f, 1, (100, 0, 200), 2)

		n = n+1
		continue #after adding the correction point, the program carries on

	vPS = [S[0] - x, S[1] - y]
	nPS = np.linalg.norm(vPS)
	uPS = vPS/nPS

	cos_a = np.dot(uPT, uPS) #for determining the magnet position through trigonometry

	l = nPS*cos_a
	L = np.sqrt(R**2-nPS**2*(1-cos_a**2))

	ex = x + uPT[0]*(l+L) #magnet position coords
	ey = y + uPT[1]*(l+L) #e for end effector

	vc.append([ex, ey, eta, l+L, nPT]) #MAG(x,y,z); MA, AB, 
	#MA and AB are inputs for time and voltage later on

	i = i+1





## RENDER
#rendering points and lines in the UI
for i in range(len(lines)-1):
	# line = cv2.line(img, lines[i], lines[i+1], (255, 209, 0), 3, 8)
	line = cv2.line(img, lines[i], (round((vc[i][0]-x0)*dim[0]/sx), -round((vc[i][1]-y0)*dim[1]/sy)), (42,40,41), 2, 4)
for i in range(n):
	ox = round((vc[i][0]-x0)*dim[0]/sx)
	oy =-round((vc[i][1]-y0)*dim[1]/sy)

	cv2.putText(img, 'o', (ox-10, oy+7), f, 1.5, (255, 209, 0), 4)
	cv2.putText(img, str(i+1), (ox -50, oy+20), f, 2, (45, 30, 190), 5)
	cv2.imshow("image", img)

cv2.waitKey(0)
# cv2.destroyAllWindows() #in order to measure time needed





## EXECUTION - dycky vrchem
#takes the vc vector as input - with magnet position and orientation
#in vc vector, there are input values for t and U for attraction part
if connected:
	for i in range(len(vc)):
		x = vc[i][0]
		y = vc[i][1]
		α = vc[i][2]
		am= vc[i][3]
		ab= vc[i][4]

		#these constants need to be tuned manually
		corr_a = 1.1
		corr_t = 1.4

		tMax = ab/12*corr_t
		a = 3.1742e-7*corr_a #for OS-01 L--, corr_a = 1
		a = 4.0090e-7*corr_a #for  S-00 L++

		dType.SetPTPCmd(api, 0, x, y, -40, α, 0)

		#ATTRACTION PART
		while dType.GetQueuedCmdMotionFinish(api)[0] == False:
			dType.dSleep(50) #waiting for arm to get in position
		else: #attraction start
			hm.set_power(1)


			U = a*am**4 #determined through experiments
			hm.set_voltage(U)
			time.sleep(tMax) #determined empirically


			hm.set_voltage(0)
			hm.set_power(0)
else: pass





## EVALUATION
if connected: 
	hm.set_power(1)
	hm.set_voltage(5) #attracts the structure until the program is turned off

print() #prints the vc vector for checking the data
print(" [ Mx :  My :  Mu :  MA :  AB ]")
print(" [----:-----:-----:-----:-----]")
for i, row in enumerate(vc):
	for j, item in enumerate(row):
		vc[i][j] =  round(item)
vc = np.array(vc,"\r")
print(vc)

cv2.imshow("image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

if connected: #returning back to safe position 
	dType.SetPTPCmd(api, 7, 0,   0, 40, 0, 1)
	dType.SetPTPCmd(api, 2, 0,-240, 20, 45, 1)
	hm.set_voltage(0)
	hm.set_power(0)


# [-142.2025604248047, -131.5725860595703, 17.249435424804688, -47.22352600097656, -137.22352600097656, 16.731689453125, 49.6535530090332, 90.0]
# [-110.8426284790039, 101.70472717285156, 29.391281127929688, 227.46176147460938, 137.46176147460938, -5.132577896118164, 45.622249603271484, 90.0]
