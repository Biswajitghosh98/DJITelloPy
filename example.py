from djitellopy import Tello
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time
import math
from collections import deque

# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 25
#initial coordinates        
py = 0.0
#pz = 0.0
px = 0.0
vx = 0.0
vy = 0.0
i = 1
sigma = 1 # FOR gaussian
#vz = 0.0
class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
    """

    def __init__(self):
        # Init pygame
        pygame.init()
        
        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()
        #initital r,p,y
        self.initial_roll=0.0
        self.initial_pitch=0.0
        self.initial_yaw=0.0
        #initial accelerations
        self.initial_ax = 0.0
        self.initial_ay = 0.0
        self.initial_az = 9999.0

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = False
        self.ptx = 0.0
        self.pty = 0.0
        #self.ptz = 0.0
        self.vx1 = 0.0
        self.vy1 = 0.0
        #self.vz1 = 0.0
        self.ax = 0.0
        self.ay = 0.0
        #self.az = 0.0

        self.agx_array = deque([])
        self.agy_array = deque([])
        self.agz_array = deque([])
        self.agx_true = 0
        self.agy_true = 0
        self.agz_true = 0

        # create update timer
        pygame.time.set_timer(USEREVENT + 1, 50)
    def get_acc(self):
        roll = math.radians(self.tello.get_roll()-self.initial_roll)
        pitch = math.radians(self.tello.get_pitch()-self.initial_pitch)
        yaw = -math.radians(self.tello.get_yaw()-self.initial_yaw)
        print("roll = ",roll,", pitch = ",pitch, ", yaw = ",yaw)
        print("agx = ",self.tello.get_agx(),", agy = ",self.tello.get_agy(), ", agz = ",self.tello.get_agz())
        Tz = np.matrix([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        Ty = np.matrix([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        Tx = np.matrix([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        T_n = Tz*Ty*Tx
        if len(self.agx_array)<8:
            self.agx_array.append(self.tello.get_agx())
            self.agy_array.append(self.tello.get_agy())
            self.agz_array.append(self.tello.get_agz())

        else:
            agx_mean = sum(self.agx_array)/8
            agy_mean = sum(self.agy_array)/8
            agz_mean = sum(self.agz_array)/8

            alphax = gaussian(self.tello.get_agxd(), agx_mean, sigma)
            alphay = gaussian(self.tello.get_agx(), agx_mean, sigma)
            alphaz = gaussian(self.tello.get_agx(), agx_mean, sigma)

            print("alphas")
            print(alphax)
            print(alphay)
            print(alphaz)

            agx_corrected = alphax*self.tello.get_agx() + (1-alphax)*agx_mean
            agy_corrected = alphay*self.tello.get_agy() + (1-alphay)*agy_mean
            agz_corrected = alphaz*self.tello.get_agz() + (1-alphaz)*agz_mean
            print("correct agx = ",agx_corrected,", correct agy = ",agy_corrected, ", correct agz = ",agz_corrected)

            self.agx_array.popleft()
            self.agx_array.append(agx_corrected)
            self.agy_array.popleft()
            self.agy_array.append(agy_corrected)
            self.agz_array.popleft()
            self.agz_array.append(agz_corrected)

            self.agx_true = agx_corrected
            self.agy_true = agy_corrected
            self.agz_true = agz_corrected

        A = np.matrix([[self.agx_true], [self.agy_true], [self.agz_true]])

        A_correct = T_n*A
        A_correct[0] = A_correct[0]-self.initial_ax
        A_correct[1] = A_correct[1]-self.initial_ay
        A_correct[2] = A_correct[2]-self.initial_az
        return A_correct

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:
            self.tello.get_battery()
            for event in pygame.event.get():
                global px,py,pz,vx,vy,vz,i
                start = time.time()
                #v_x =self.tello.get_vgx()
                #v_y =self.tello.get_vgy()
                #v_z =self.tello.get_vgz()
                #ax = self.tello.get_agx()
                #ay = self.tello.get_agy()
                #az = self.tello.get_agz()

                A = self.get_acc()

                end = time.time()
                #start1 = time.time()
                self.ptx = px
                self.pty = py
                #self.ptz = pz
                self.vx1 = A[0]*(end-start)
                self.vy1 = A[1]*(end-start)
                #self.vz1 = self.az*(end-start)
                vx = vx + self.vx1
                vy = vy + self.vy1
                #vz = vz + self.vz1
                px = self.ptx + (vx*1000)*((end-start)*1000)/1000000.0
                py = self.pty + (vy*1000)*((end-start)*1000)/1000000.0
                #pz = self.ptz + vz*(end-start)
                print("VX :   "+ str(vx) + "    VY :   " + str(vy))
                print("X :   "+ str(px) + "    Y :   " + str(py))
                print("AX :   "+ str(A[0]) + "    AY :   " + str(A[1])+"    AZ :    " + str(A[2]))
                if event.type == USEREVENT + 1:
                	#end1 = time.time()
                	#print("Extra time lag : "+str(end1-start1))
                	self.update()
                elif event.type == QUIT:
                	#end1 = time.time()
                	#print("Extra time lag : "+str(end1-start1))
                	should_stop = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                    	#end1 = time.time()
                    	#print("Extra time lag : "+str(end1-start1))
                    	should_stop = True
                    else:
                    	#end1 = time.time()
                    	#print("Extra time lag : "+str(end1-start1))
                        self.keydown(event.key)
                elif event.type == KEYUP:
                	#end1 = time.time()
                	#print("Extra time lag : "+str(end1-start1))
                	self.keyup(event.key)

            if frame_read.stopped:
                frame_read.stop()
                break

            self.screen.fill([0, 0, 0])
            frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)
            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

        # Call it always before finishing. I deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw counter clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        #if self.send_rc_control:
        #    self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
        #                               self.yaw_velocity)

def gaussian(x, mean, sigma):
    g = np.exp(-0.5*((x-mean)/sigma)**2)
    return g

def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
