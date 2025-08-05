import sys, os
import math, time, json
import numpy as np
import starkit
from math import pi
from Robots.class_Robot_Roki_2 import Robot
current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
current_work_directory += '/'
with open(current_work_directory + "simulator_lib_directory.txt", "r") as f:
    simulator_lib_directory = f.read()
simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
sys.path.append(simulator_lib_directory)
import sim
from Soccer.Localisation.class_Glob import Glob, Variables_4_Walk
from multiprocessing import Array, Value, Process, Queue
import threading

def motion_sim_as_process(var, Dummy_1Data, trigger_queue, trigger_release, end_walking):
    class Motion(Robot):
        def __init__(self):
            super().__init__()
            #glob = Glob(var.SIMULATION.value, current_work_directory)
            print('SIMULATION : ', var.SIMULATION.value)
            self.sim = sim
            #self.glob = glob
            #self.params = self.glob.params
            self.slowTime   = 0.0             # seconds
            if var.SIMULATION.value == 0: self.slowTime   = 0.5
            self.simThreadCycleInMs = 20
            #self.motion_shift_correction_x = -self.glob.params['MOTION_SHIFT_TEST_X'] / 21
            #self.motion_shift_correction_y = -self.glob.params['MOTION_SHIFT_TEST_Y'] / 21
            #self.first_step_yield = self.glob.first_step_yield
            #self.cycle_step_yield = self.glob.cycle_step_yield
            #self.side_step_right_yield = self.glob.side_step_right_yield
            #self.side_step_left_yield = self.glob.side_step_left_yield
            #self.imu_drift_speed = math.radians(self.glob.params['IMU_DRIFT_IN_DEGREES_DURING_6_MIN_MEASUREMENT'])/ 360
            self.stepLength = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
            self.sideLength = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
            self.rotation = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.
            #self.first_Leg_Is_Right_Leg = True
            self.initPoses = 400//self.simThreadCycleInMs
            self.exitFlag = 0
            self.falling_Flag = 0
            self.body_euler_angle ={}
            self.local = 0 # Local
            self.direction_To_Attack = 0
            self.activePose = []
            self.xtr = 0
            self.ytr = -self.d10   #-53.4
            self.ztr = -self.gaitHeight
            self.xr = 0
            self.yr = 0
            self.zr = -1
            self.wr = 0
            self.xtl = 0
            self.ytl = self.d10   # 53.4
            self.ztl = -self.gaitHeight
            self.xl = 0
            self.yl = 0
            self.zl = -1
            self.wl = 0
            self.tors_angle = 0
            self.fase_offset = 0.7
            self.xtl0 = 0
            self.xtr0 = 0
            self.dx0_typical = 0
            self.dy0_typical = 0
            self.alpha01 = 0
            self.lateral_offset = 0
            self.ugol_torsa = 0.4
            self.dobavka_x_ot_torsa = 0
            self.euler_angle = {}
            self.robot_In_0_Pose = True
            self.keep_hands_up = False
            self.clientID = None
            self.Dummy_1Handle = None
            self.start_point_for_imu_drift = time.perf_counter()
            import msvcrt as ms
            self.ms = ms
            self.jointHandle = []
            self.trims = []


        #-------------------------------------------------------------------------------------------------------------------------------
        def trigger(self, id):
            trigger_queue.put(id)     # send trigger
            while True:
                message = trigger_release.get(block=True)                 #check if release related to current request
                if message == id: break
                else: trigger_release.put(message)
                time.sleep(0.01)
        
        def sim_Start(self):
            self.clientID=self.sim.simxStart('127.0.0.1',-19998,True,True,5000, 2) # Connect to V-REP
            if self.clientID!=-1:
                print ('Connected to remote API server')
            else:
                print ('Failed connecting to remote API server')
                print ('Program ended')
                exit(0)
            returnCode, self.Dummy_1Handle = self.sim.simxGetObjectHandle(self.clientID, '/Dummy1', self.sim.simx_opmode_blocking)
            returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
            for i in range(len(self.ACTIVEJOINTS)):
                returnCode, handle= self.sim.simxGetObjectHandle(self.clientID, '/'+self.ACTIVEJOINTS[i], self.sim.simx_opmode_blocking)
                self.jointHandle.append(handle)
                returnCode, position= self.sim.simxGetJointPosition(self.clientID, handle, self.sim.simx_opmode_blocking)
                self.trims.append(position)
                self.activePose.append(position)
            if var.SIMULATION.value == 3:
                self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_streaming)

        def imu_body_yaw(self):
            yaw = self.body_euler_angle['yaw']
            yaw = self.norm_yaw(yaw)
            return yaw

        def norm_yaw(self, yaw):
            yaw %= 2 * pi
            if yaw > pi:  yaw -= 2* pi
            if yaw < -pi: yaw += 2* pi
            return yaw

        def from_vrep_quat_to_conventional_quat(self, quaternion):
                x,y,z,w = quaternion
                return [w,x,y,z]

        def quaternion_to_euler_angle(self, quaternion):
            euler_angle = {}
            w,x,y,z = quaternion
            ysqr = y*y
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + ysqr)
            X = math.atan2(t0,t1)
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            Y = math.asin(t2)
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (ysqr + z * z)
            Z = math.atan2(t3,t4)
            euler_angle['yaw'] = Z
            euler_angle['pitch'] = Y
            euler_angle['roll'] = X
            return euler_angle

        def falling_Test(self):
            self.refresh_Orientation()
            if var.SIMULATION.value == 0 or var.SIMULATION.value == 1 or var.SIMULATION.value == 3:
                key = 0
                if self.ms.kbhit():
                    key = self.ms.getch()
                if key == b'p' :
                    #self.lock.acquire()
                    if var.SIMULATION.value == 3:
                        self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                    key = 0
                    while (True):
                        if self.ms.kbhit():
                            key = self.ms.getch()
                        if key == b'p':
                            #self.lock.release()
                            if var.SIMULATION.value == 3:
                                self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                            key = 0
                            break
                if key == b's': # or self.transfer_Data.stop_Flag:
                    print('Simulation STOP by keyboard')
                    #self.transfer_Data.stop += 1
                    self.sim_Stop()
                    self.sim_Disable()
                    #while True:
                    #    if self.transfer_Data.stop == self.numberOfRobots:
                    #        self.sim_Disable()
                    #        sys.exit(0)
                    #    time.sleep(0.1)
                    self.falling_Flag = 3
                    #self.transfer_Data.stop_Flag = True
                    return self.falling_Flag
                #returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                #Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
                #self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                if (self.body_euler_angle['pitch']) > 0.785:
                    self.falling_Flag = 1                   # on stomach
                    self.simulateMotion(name = self.model + '_Get_UP_Stomach')
                if (self.body_euler_angle['pitch']) <  -0.785:
                    self.falling_Flag = -1                  # face up
                    self.simulateMotion(name = self.model + '_Get_UP_Face_Up')
                if (self.body_euler_angle['roll']) > 0.785:
                    self.falling_Flag = -2                  # on right side
                    self.simulateMotion(name = 'Get_Up_Right')
                if -135< (self.body_euler_angle['roll']) < -0.785:
                    self.falling_Flag = 2                   # on left side
                    self.simulateMotion(name = 'Get_Up_Left')
            return self.falling_Flag

        def computeAlphaForWalk(self, hands_on = True ):
            angles =[]
            anglesR=[]
            anglesL=[]
            anglesR = starkit.alpha_calculation(self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, self.SIZES, self.LIMALPHA)
            anglesL = starkit.alpha_calculation(self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,self.wl, self.SIZES, self.LIMALPHA)
            #else:
            #    anglesR = self.al.compute_Alpha_v3(self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, self.SIZES, self.LIMALPHA)
            #    anglesL = self.al.compute_Alpha_v3(self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,self.wl, self.SIZES, self.LIMALPHA)
            if len(anglesR)>1:
                for i in range(len(anglesR)):
                    if len(anglesR)==1: break
                    if anglesR[0][2]<anglesR[1][2]: anglesR.pop(1)
                    else: anglesR.pop(0)
            elif len(anglesR)==0:
                return[]
            if len(anglesL)>1:
                for i in range(len(anglesL)):
                    if len(anglesL)==1: break
                    if anglesL[0][2]<anglesL[1][2]: anglesL.pop(1)
                    else: anglesL.pop(0)
            elif len(anglesL)==0:
                return[]
            #if self.first_Leg_Is_Right_Leg == 1:
            if var.first_Leg_Is_Right_Leg.value == 1:
                for j in range(6): angles.append(anglesR[0][j])
                if hands_on: angles.append(1.973)
                #if hands_on: angles.append(1.507 +(-0.147 - self.xtl/30))
                else: angles.append(0.0)
                angles.append(0.0)
                angles.append(0.0)
                if hands_on: angles.append(-0.147 - self.xtl/114) 
                #if hands_on: angles.append(-0.147 - self.xtl/30)
                else: angles.append(0.0)
                angles.append(0.0)
                for j in range(6): angles.append(-anglesL[0][j])
                if hands_on: angles.append(-1.973)
                #if hands_on: angles.append(-1.507 + 0.147 + self.xtr/30)
                else: angles.append(0.0)
                angles.append(0.0)
                angles.append(0.0)
                if hands_on: angles.append(0.147 + self.xtr/114)
                #if hands_on: angles.append(0.147 + self.xtr/30)
                else: angles.append(0.0)
            else:
                for j in range(6): angles.append(anglesL[0][j])
                if hands_on: angles.append(1.973)    #(1.745)
                else: angles.append(0.0)
                angles.append(0.0)
                angles.append(0.0)
                if hands_on: angles.append(-0.147 - self.xtr/114)  #(0.524 - self.xtr/57.3)
                else: angles.append(0.0)
                angles.append(0.0)                                  # Tors
                for j in range(6): angles.append(-anglesR[0][j])
                if hands_on: angles.append(-1.973)  #(-1.745)
                else: angles.append(0.0)
                angles.append(0.0)
                angles.append(0.0)
                if hands_on: angles.append(0.147 + self.xtl/114)  #(-0.524 + self.xtl/57.3)
                else: angles.append(0.0)
            self.activePose = angles
            return angles

        def walk_Initial_Pose(self, amplitude = 24):
            self.robot_In_0_Pose = False
            if not self.falling_Test() == 0:
                #self.local.quality =0
                if self.falling_Flag == 3: print('STOP!')
                else: print('FALLING!!!', self.falling_Flag)
                return[]
            self.xtr = self.xtl = 0
            framestep = self.simThreadCycleInMs//10
            for j in range (self.initPoses):
                self.ztr = self.ztr0 - j*(self.ztr0 + self.gaitHeight)/self.initPoses
                self.ztl = self.ztl0 - j*(self.ztl0+self.gaitHeight)/self.initPoses
                # self.ytr = -self.d10 - j*self.amplitude/2 /self.initPoses
                # self.ytl =  self.d10 - j*self.amplitude/2 /self.initPoses
                self.ytr = -self.d10 - j* amplitude /self.initPoses
                self.ytl =  self.d10 - j* amplitude /self.initPoses
                angles = self.computeAlphaForWalk()
                #if not self.falling_Flag ==0: return
                if len(angles)==0:
                    self.exitFlag = self.exitFlag +1
                else:
                    if var.SIMULATION.value == 1 or var.SIMULATION.value  == 0 or var.SIMULATION.value == 3:
                        if var.SIMULATION.value == 3: self.wait_sim_step()
                        for i in range(len(angles)):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            if var.SIMULATION.value == 1 or var.SIMULATION.value == 3:
                               returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3], #+self.trims[i],
                                            self.sim.simx_opmode_oneshot)
                            elif var.SIMULATION.value == 0:
                                returnCode = self.sim.simxSetJointPosition(self.clientID,
                                             self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3], #+self.trims[i],
                                             self.sim.simx_opmode_oneshot)
                        if var.SIMULATION.value == 1 or var.SIMULATION.value  == 0 or var.SIMULATION.value == 3:
                            time.sleep(self.slowTime)
                        if var.SIMULATION.value == 1:
                            #self.sim.simxSynchronousTrigger(self.clientID)
                            #print('trigger request_1')
                            self.trigger('hard_walk_initial_pose')
                            
                                              

        def walk_Cycle(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles, half = 0):
            self.robot_In_0_Pose = False
            if not self.falling_Test() == 0:
                #self.local.quality =0
                if self.falling_Flag == 3: print('STOP!')
                else: print('FALLING!!!', self.falling_Flag)
                return[]
            self.stepLength = stepLength + var.motion_shift_correction_x.value
            self.sideLength = sideLength - var.motion_shift_correction_y.value
            self.rotation = math.degrees(rotation)
            #tmp1 = self.first_Leg_Is_Right_Leg
            #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
            #else: self.first_Leg_Is_Right_Leg = True
            if self.rotation <= 0: 
                rotation = -self.rotation/222 * 0.23 / var.rotation_yield_right.value
            else:
                rotation = -self.rotation/222 * 0.23 / var.rotation_yield_left.value
            if rotation > 0.125 : rotation = 0.125
            if rotation < -0.125 : rotation = -0.125
            alpha = 0
            if self.fr1 == 0:
                alpha01 = math.pi
            else:
                alpha01 = math.pi/self.fr1*2
            frameNumberPerCycle = 2*self.fr1+2*self.fr2
            framestep = self.simThreadCycleInMs//10
            xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) * 1.5     # 1.5 - podgon
            xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
            dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
            dy0_typical = self.sideLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion sideways per framestep
            xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
            # correction of body tilt forward
            if stepLength < 0:
                self.xr, self.xl = var.body_tilt_at_walk_backwards.value, var.body_tilt_at_walk_backwards.value
            else:
                self.xr, self.xl = var.body_tilt_at_walk.value, var.body_tilt_at_walk.value   #
            # correction of sole skew depending on side angle of body when step pushes land
            self.yr, self.yl = - var.sole_lading_skew.value, var.sole_lading_skew.value
            fase_offset = 0.7 #1.57  #0.7
            #order = [10.7, 0, -10.5, -16, -10.7, 0, 10.5, 16]
            order = []
            pos = int(self.amplitude/2)
            for _ in range(self.fr1):
                pos -= int(self.amplitude/self.fr1)
                order.append(pos)
            #order = [8, 0, -8, -16, -8, 0, 8, 16]
            for iii in range(0,frameNumberPerCycle,framestep):
                if half == 1 and iii == (frameNumberPerCycle/2 - framestep): break 
                if 0<= iii <self.fr1 :                                              # FASA 1
                    #alpha = alpha01 * (iii/2+ fase_offset*framestep)
                    #alpha = alpha01 * iii/2
                    #S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                    S = order[iii + framestep - 1] * (self.amplitude + self.sideLength ) / self.amplitude
                    self.ytr = S - self.d10 + self.sideLength/2
                    self.ytl = S + self.d10 + self.sideLength/2
                    self.ztl = -self.gaitHeight
                    self.ztr = -self.gaitHeight
                    if cycle ==0: continue
                    else: dx0 = dx0_typical
                    self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                    self.xtr = xtr0 - dx0 - dx0 * iii/framestep

                if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :                     # FASA 3
                    #alpha = alpha01 * ((iii-self.fr2)/2+ fase_offset*framestep)
                    #alpha = alpha01 * (iii-self.fr2)/2
                    #S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                    S = - order[iii - self.fr1 - self.fr2 + framestep - 1] * (self.amplitude + self.sideLength ) / self.amplitude
                    self.ytr = S - self.d10 - self.sideLength/2
                    self.ytl = S + self.d10 + self.sideLength/2
                    self.ztl = -self.gaitHeight
                    self.ztr = -self.gaitHeight
                    dx0 = dx0_typical
                    self.xtl -= dx0
                    self.xtr -= dx0

                if self.fr1<= iii <self.fr1+self.fr2:                               # FASA 2
                    self.ztr = -self.gaitHeight + self.stepHeight
                    if cycle ==0:
                        dx = self.stepLength/(self.fr2- 2 * framestep)*framestep/2
                        dx0 = dx0_typical
                        dy = self.sideLength/self.fr2*framestep
                        dy0 = dy0_typical
                    else:
                        dx = self.stepLength/(self.fr2- 2 * framestep)*framestep #* 0.75
                        dx0 = dx0_typical
                        dy = self.sideLength/self.fr2*framestep
                        dy0 = dy0_typical
                    if iii==self.fr1:
                        self.xtr -= dx0
                        #self.ytr = S - 64 + dy0
                        self.ytr = S - self.d10 + dy0
                        self.ztr = -self.gaitHeight + self.stepHeight/2
                    elif iii == (self.fr1 +self.fr2 - framestep):
                        self.xtr -= dx0
                        self.ytr = S - self.d10 + 2*dy0 - self.sideLength
                    else:
                        self.xtr += dx
                        self.ytr = S - 64 + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                        self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                    self.xtl -= dx0
                    self.ytl += dy0

                if 2*self.fr1+self.fr2<= iii :                                         # FASA 4
                    self.ztl = -self.gaitHeight + self.stepHeight
                    #if cycle == number_Of_Cycles - 2:
                    #    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    #        self.ztl = -self.gaitHeight
                    if cycle == number_Of_Cycles - 1:
                        dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                        dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2- 2 * framestep) *framestep / 1.23076941   # 1.23076941 = podgon
                        if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                            self.ztl = -self.gaitHeight
                            self.ytl = S + self.d10
                    else:
                        dx = self.stepLength/(self.fr2- 2 * framestep) *framestep # * 0.75
                        dx0 = dx0_typical
                        dy = self.sideLength/(self.fr2- 2 * framestep) *framestep
                        dy0 = dy0_typical
                    if iii== (2*self.fr1 + self.fr2 ):
                        self.xtl -= dx0
                        #self.ytl = S + 64 + dy0
                        self.ytl = S + self.d10 + dy0
                        self.ztl = -self.gaitHeight + self.stepHeight/2
                    elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                        self.xtl -= dx0
                        self.ytl = S + self.d10 + 2*dy0 - self.sideLength
                    else:
                        self.xtl += dx
                        self.ytl = S + 64 + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                        self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                    self.xtr -= dx0
                    self.ytr += dy0
                angles = self.computeAlphaForWalk()
                #print('iii = ', iii, 'xtr =', round(self.xtr,2), 'xtl =', round(self.xtl, 2))
                if not self.falling_Flag ==0: return
                if len(angles)==0:
                    print('bad_ik_calc:', 'iii = ', iii, 'xtr:', self.xtr, 'ytr:', self.ytr, 'ztr:', self.ztr, 'xtl:', self.xtl, 'ytl:', self.ytl, 'ztl:', self.ztl )
                    self.exitFlag = self.exitFlag +1
                else:
                    if var.SIMULATION.value == 1 or var.SIMULATION.value  == 0 or var.SIMULATION.value == 3:
                        if var.SIMULATION.value == 3: self.wait_sim_step()
                        for i in range(len(angles)):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            if var.SIMULATION.value == 1 or var.SIMULATION.value == 3:
                               returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3],#+self.trims[i],
                                            self.sim.simx_opmode_oneshot)
                            elif var.SIMULATION.value == 0:
                               returnCode = self.sim.simxSetJointPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3], #+self.trims[i],
                                            self.sim.simx_opmode_oneshot)
                        returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                        Dummy_1Data.put(Dummy_1position) 
                        if var.SIMULATION.value == 1:
                            #self.sim.simxSynchronousTrigger(self.clientID)
                            #print('trigger request_2')
                            self.trigger('hard_walk_cycle')
                    #self.refresh_Orientation()
            # returning xr, xl, yr, yl to initial value
            self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
            var.coord_shift_x.value = var.cycle_step_yield.value * stepLength/64/1000
            # if var.first_Leg_Is_Right_Leg.value :
            #     var.coord_shift_y.value = -var.side_step_right_yield.value * sideLength/20/1000
            # else: var.coord_shift_y.value = var.side_step_left_yield.value * sideLength/20/1000
            var.coord_shift_y.value = -var.side_step_right_yield.value * sideLength/20/1000 * var.first_Leg_Is_Right_Leg.value
            if half ==1: 
                var.coord_shift_x.value *= 0.5
                var.coord_shift_y.value *= 0.5

        def normalize_rotation(self, yaw):
            if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
            if yaw > math.pi : yaw -= (2 * math.pi)
            if yaw < -math.pi : yaw += (2 * math.pi)
            if yaw > 0.3 : yaw = 0.3
            if yaw < -0.3 : yaw = -0.3
            return yaw

        def get_rotation(self):
            self.refresh_Orientation()
            rotation = var.direction_0.value - self.imu_body_yaw()
            #print('direction_0 :', var.direction_0.value, 'imu_body_yaw: ', self.imu_body_yaw())
            if var.first_Leg_Is_Right_Leg.value == -1:
                rotation *= -1
            return self.normalize_rotation(rotation)

        def walk_Final_Pose(self, respect_body_tilt = False):
            self.robot_In_0_Pose = False
            if not self.falling_Test() == 0:
                #self.local.quality =0
                if self.falling_Flag == 3: print('STOP!')
                else: print('FALLING!!!', self.falling_Flag)
                return[]
            framestep = self.simThreadCycleInMs//10
            if respect_body_tilt: 
                xr, xl = self.xr, self.xl
                self.xr, self.xl = var.body_tilt_at_walk.value, var.body_tilt_at_walk.value
            for j in range (self.initPoses):
                if var.SIMULATION.value == 5: start1 = time.perf_counter()
                self.ztr = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/self.initPoses
                self.ztl = -self.gaitHeight - (j+1)*(233.0-self.gaitHeight)/self.initPoses
                self.ytr = -self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
                self.ytl =  self.d10 - (self.initPoses-(j+1))*self.amplitude/2 /self.initPoses
                if j == self.initPoses - 1:
                    angles = self.computeAlphaForWalk(hands_on = False)
                else: angles = self.computeAlphaForWalk()
                #if not self.falling_Flag ==0: return
                if len(angles)==0:
                    self.exitFlag = self.exitFlag +1
                else:
                    if var.SIMULATION.value == 1 or var.SIMULATION.value  == 0 or var.SIMULATION.value == 3:
                        if var.SIMULATION.value == 3: self.wait_sim_step()
                        for i in range(len(angles)):
                            if self.keep_hands_up:
                                if i in self.hand_joints : continue
                            if var.SIMULATION.value == 1 or var.SIMULATION.value == 3:
                               returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3],#+self.trims[i],
                                            self.sim.simx_opmode_oneshot)
                            elif var.SIMULATION.value == 0:
                               returnCode = self.sim.simxSetJointPosition(self.clientID,
                                            self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3], #+self.trims[i],
                                            self.sim.simx_opmode_oneshot)
                        if var.SIMULATION.value == 1:
                            #self.sim.simxSynchronousTrigger(self.clientID)
                            #print('trigger request_3')
                            self.trigger('hard_walk_final_pose')
            if respect_body_tilt: 
                self.xr, self.xl = xr, xl

        def refresh_Orientation(self):
            returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
            Dummy_1quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_1quaternion)
            #print('Dummy_1quaternion: ', Dummy_1quaternion)
            self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
            self.body_euler_angle['yaw'] -= self.direction_To_Attack
            self.body_euler_angle['yaw'] += var.imu_drift_speed.value * (time.perf_counter() - self.start_point_for_imu_drift)

    def slide_order():
        if var.number_Of_Cycles.value == 4:
            var.stepLength_0.value = var.stepLength_1.value
            var.sideLength_0.value = var.sideLength_1.value
            var.direction_0.value = var.direction_1.value 
            var.stepLength_1.value = var.stepLength_2.value
            var.sideLength_1.value = var.sideLength_2.value
            var.direction_1.value = var.direction_2.value
            var.stepLength_2.value = var.stepLength_3.value
            var.sideLength_2.value = var.sideLength_3.value
            var.direction_2.value = var.direction_3.value
            var.number_Of_Cycles.value = 3
        elif var.number_Of_Cycles.value == 3:
            var.stepLength_0.value = var.stepLength_1.value
            var.sideLength_0.value = var.sideLength_1.value
            var.direction_0.value = var.direction_1.value 
            var.stepLength_1.value = var.stepLength_2.value
            var.sideLength_1.value = var.sideLength_2.value
            var.direction_1.value = var.direction_2.value
            var.number_Of_Cycles.value = 2
        elif var.number_Of_Cycles.value == 2:
            var.stepLength_0.value = var.stepLength_1.value
            var.sideLength_0.value = var.sideLength_1.value
            var.direction_0.value = var.direction_1.value 
            var.number_Of_Cycles.value = 1
        elif var.number_Of_Cycles.value == 1:
            var.stepLength_0.value = 0
            var.sideLength_0.value = 0
            var.number_Of_Cycles.value = 0
        elif var.number_Of_Cycles.value == 0:
            var.stepLength_0.value = 0
            var.sideLength_0.value = 0

    motion = Motion()
    motion.sim_Start()
    while True:
        #print('number_Of_Cycles: ', var.number_Of_Cycles.value)
        if var.number_Of_Cycles.value > 0:
            var.first_Leg_Is_Right_Leg.value = - int(math.copysign(1, var.sideLength_0.value))
            motion.walk_Initial_Pose()
            stepLength1 = min(20, var.stepLength_0.value)
            rotation = motion.get_rotation()
            motion.walk_Cycle(stepLength1, abs(var.sideLength_0.value), rotation, 0, 2, half = var.half.value)
            slide_order()
            var.half.value = 0
            while True:
                if var.number_Of_Cycles.value < 1 : break
                rotation = motion.get_rotation()
                motion.walk_Cycle(var.stepLength_0.value, abs(var.sideLength_0.value), rotation, 1, 3, half = var.half.value)
                slide_order()
                var.half.value = 0
            rotation = motion.get_rotation()
            motion.walk_Cycle(var.stepLength_0.value, abs(var.sideLength_0.value), rotation, 1, 2, half = var.half.value)
            slide_order()
            var.half.value = 0
            motion.walk_Final_Pose()
            end_walking.put('end')
        time.sleep(0.1)


def motion_sim_as_process_launch(glob, motion):
    if not glob.hardcode_walking: return None
    
    var = Variables_4_Walk()
    var.stepLength_0 = Value('f',0)
    var.sideLength_0 = Value('f',0)
    var.direction_0 = Value('f',0)
    var.stepLength_1 = Value('f',0)
    var.sideLength_1 = Value('f',0)
    var.direction_1 = Value('f',0)
    var.stepLength_2 = Value('f',0)
    var.sideLength_2 = Value('f',0)
    var.direction_2 = Value('f',0)
    var.stepLength_3 = Value('f',0)
    var.sideLength_3 = Value('f',0)
    var.direction_3 = Value('f',0)
    var.rotation = Value('f',0)
    var.gaitHeight = Value('f',0)
    var.stepHeight = Value('f',0)
    var.fr1 = Value('i',0)
    var.fr2 = Value('i',0)
    var.amplitude = Value('f',0)
    var.body_tilt_at_walk = Value('f',0)
    var.body_tilt_at_walk_backwards = Value('f',0)
    var.sole_lading_skew = Value('f',0)
    var.body_tilt_at_kick = Value('f',0)
    var.number_Of_Cycles = Value('i', 0)
    var.SIMULATION = Value('i', glob.SIMULATION)
    var.coord_shift_x = Value('f',0)
    var.coord_shift_y = Value('f',0)
    var.first_Leg_Is_Right_Leg = Value('i', 1)
    var.half = Value('i', 0)
    var.motion_shift_correction_x = Value('f',0)
    var.motion_shift_correction_y = Value('f',0)
    var.first_step_yield = Value('f',0)
    var.cycle_step_yield = Value('f',0)
    var.side_step_right_yield = Value('f',0)
    var.side_step_left_yield = Value('f',0)
    var.rotation_yield_right = Value('f',0)
    var.rotation_yield_left = Value('f',0)
    var.imu_drift_speed = Value('f',0)
    Dummy_1Data = Queue()
    queue1  = Queue()
    queue2  = Queue()
    end_walking = Queue()
    p1 = threading.Thread(target = motion_sim_as_process, args = (var, Dummy_1Data, motion.trigger_queue, motion.trigger_release, end_walking), daemon = True)
    p1.start()
    motion.var = var
    #t1 = threading.Thread(target = motion.trigger_Enabler, args = (queue1, queue2), daemon = True)
    #t1.start()
    return var, Dummy_1Data, end_walking



if __name__=="__main__":
    print('This is not main module!')


