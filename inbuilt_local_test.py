from djitellopy import Tello
from time import sleep     
import numpy as np
import cv2
import time

tello = Tello()
tello.connect()
tello.streamoff()
tello.streamon()

# Main 
if __name__=='__main__':

	frame_read = tello.get_frame_read()

	while True:
		start = time.time()
		# print(tello.get_current_state_all())
		# print(tello.get_pitch())
		# print(tello.get_roll())
		# print(tello.get_yaw())
		####
		print("Velocity in x is {}".format(tello.get_vgx()))
		print("Velocity in y is {}".format(tello.get_vgy()))
		print("Accelaration in x is {}".format(tello.get_agx()))
		print("Accelaration in y is {}".format(tello.get_agy()))
		print("Acceleration in z is {}".format(tello.get_agz()))
		print(tello.get_h())
		print(tello.get_bat())
		
		cv2.imshow("Input Image", frame_read.frame)

		key = cv2.waitKey(5) & 0xFF
		if key == ord("w"):
			rcOut[1] = 50
		elif key == ord("a"):
			rcOut[0] = -50
		elif key == ord("s"):
			rcOut[1] = -50
		elif key == ord("d"):
			rcOut[0] = 50
		elif key == ord("u"):
			rcOut[2] = 50
		elif key == ord("j"):
			rcOut[2] = -50
		elif key == ord("q"):
			break
		elif key == ord("t"):
			tello.takeoff() 
			sleep(5)   
		elif key == ord("l"):
			tello.land()
		else:
			rcOut = [0,0,0,0]

		# print self.rcOut
		tello.send_rc_control(int(rcOut[0]),int(rcOut[1]),int(rcOut[2]),int(rcOut[3]))
		rcOut = [0,0,0,0]
		end = time.time()
		print("T L : "+str(end-start))

tello.end()

cv2.destroyAllWindows()