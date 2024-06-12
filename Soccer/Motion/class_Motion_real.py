#  Walking engine for Starkit ROKI for RPI
#  Copyright STARKIT Soccer team of MIPT

import math, time, json
from math import sin, cos, pi, atan2, atan
import random
import threading
import threading
import Soccer.Vision.reload as re
from Soccer.Motion.class_Motion import Motion
#from ball_Approach_Steps_Seq import *

def uprint(*text):
    print(*text )

class Motion_real(Motion):

    def __init__(self, glob, vision):
        super().__init__(glob, vision)
        self.stop_control_Head_motion = threading.Event()
        self.control_Head_motion_thread = threading.Thread(target = self.control_Head_motion, args=(self.stop_control_Head_motion,))
        self.control_Head_motion_thread.setDaemon(True)
        #self.control_Head_motion_thread.start()

    def head_Tilt_Calibration(self):
            # Калибрация Наклона камеры. Установить мяч на расстоянии (100 см)по низу мяча.
            # Полученная величина наклона камеры эквивалентна (69) градуса от вертикали.
            # Вторая позиция головы 23 градуса к вертикали отличается от первой на 1155
        return_value = 0
        if self.glob.SIMULATION == 2:
            uprint(' head_tilt_calibr')
            i= 400
            a = True
            self.kondo.setUserParameter(19,0)
            while(a):
                if (i < -1500) : a=False
                #clock.tick()
                i=i-1
                uprint ('i =', i)
                b=self.kondo.setUserParameter(20,i)
                for j in range(5):
                    img = self.sensor.snapshot().lens_corr(strength = 1.45, zoom = 1.0)
                    for blob in img.find_blobs([self.vision.TH['orange ball']['th']], pixels_threshold=20, area_threshold=20, merge=True):
                        #if blob.roundness() > 0.5:
                        img.draw_rectangle(blob.rect())
                        img.draw_cross(blob.cx(), blob.cy())
                        uprint('blob.cy() =', blob.cy())
                        #if (blob.y()+ blob.h()) <=120 : a=False
                        if blob.cy() <=self.params['CAMERA_VERTICAL_RESOLUTION'] / 2 : a=False
                        return_value = blob.cy()

        else:
            #import reload as re
            i= 200
            a = True
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , 0*self.ACTIVESERVOS[21][3], self.sim.simx_opmode_oneshot) #head pan to zero
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , 0*self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot) #head tilt to zero
            while(a):
                if (i < -1500) : a=False
                #clock.tick()
                i=i-1
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , i * self.TIK2RAD * self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot)
                self.sim_simxSynchronousTrigger(self.clientID)
                img1 = self.vision_Sensor_Get_Image()
                img = re.Image(img1)
                for blob in img.find_blobs([self.vision.TH['orange ball']['th']],
                                            pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                            area_threshold=self.vision.TH['orange ball']['area'],
                                            merge=True):
                    img.draw_rectangle(blob.rect())
                    if blob.cy()==self.params['CAMERA_VERTICAL_RESOLUTION'] / 2 : a=False
                    return_value = blob.cy()
                self.vision_Sensor_Display(img.img)
        if (self.params['CAMERA_VERTICAL_RESOLUTION'] / 2 - 4) < return_value <=self.params['CAMERA_VERTICAL_RESOLUTION'] / 2 :
            self.neck_calibr = i
            self.neck_play_pose = int(self.neck_calibr - self.params['CAMERA_VERTICAL_RESOLUTION'] / 2 * self.params['APERTURE_PER_PIXEL_VERTICAL'] / 0.03375)
            self.refresh_Orientation()
            data = {"neck_calibr": self.neck_calibr, "neck_play_pose": self.neck_play_pose, "head_pitch_with_horizontal_camera": self.euler_angle['pitch']}
            if self.glob.SIMULATION == 5:
                with open(self.glob.current_work_directory + "Init_params/Real/Real_calibr.json", "w") as f:
                    json.dump(data, f)
            else:
                with open(self.glob.current_work_directory + "Init_params/Sim/" + "Sim_calibr.json", "w") as f:
                    json.dump(data, f)
            return True
        else: return False

    def seek_Ball(self, fast_Reaction_On):                  # seeks ball in 360 dergree one time
        a, course, distance = self.seek_Ball_In_Pose(fast_Reaction_On)
        if a == True: return a, course, distance
        else:
            target_course1 = self.euler_angle['yaw'] +math.pi
            self.turn_To_Course(target_course1)
            a, course, distance = self.seek_Ball_In_Pose(fast_Reaction_On)
            if a == True: return a, course, distance
            else:
                target_course = target_course1 -170
                self.turn_To_Course(target_course)
                return False, 0, 0

    def seek_Ball_In_Pose(self, fast_Reaction_On, penalty_Goalkeeper = False, with_Localization = True, very_Fast = False, first_look_point= None):
        self.local.correct_yaw_in_pf()
        if self.robot_In_0_Pose == False:
            if self.glob.SIMULATION == 5:
                self.play_Soft_Motion_Slot(name = 'Initial_Pose')
            else:
                self.simulateMotion(name = 'Initial_Pose')
            self.robot_In_0_Pose = True
        variants = []
        if first_look_point == None:
            course_to_ball = 0
            tilt_to_ball = (-self.neck_play_pose - self.neck_calibr) * self.TIK2RAD 
        else:
            course_to_ball = self.norm_yaw(math.atan2((first_look_point[1] - self.glob.pf_coord[1]), (first_look_point[0] - self.glob.pf_coord[0])) - self.glob.pf_coord[2])
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        head_pose_seq = [2,3,1]
        for i in range(len(head_pose_seq)+1):
            if  i == 0:
                self.neck_pan = -int(course_to_ball/self.TIK2RAD)
                if abs(self.neck_pan) > 1333: self.neck_pan = int(math.copysign(1333, self.neck_pan))
                self.neck_tilt = c
            else:
                x = head_pose[head_pose_seq[i-1]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
            if not self.falling_Test() == 0:
                self.local.quality =0
                if self.falling_Flag == 3: uprint('STOP!')
                else: uprint('FALLING!!!', self.falling_Flag)
                return False, 0, 0, [0, 0]
            if self.glob.SIMULATION == 5:
                self.head_Return(self.neck_pan, self.neck_tilt)
            else:
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , self.neck_pan * self.TIK2RAD * self.ACTIVESERVOS[21][3], self.sim.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_tilt * self.TIK2RAD * self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(20):
                    self.sim_simxSynchronousTrigger(self.clientID)
            #self.refresh_Orientation()
            a, course, dist, blob = self.vision.seek_Ball_In_Frame(with_Localization)
            if a == True: 
                variants.append ((course, dist *1000))
            if fast_Reaction_On == True and a== True: break
        course = 0
        distance = 0
        if len(variants)>0:
            for i in range (len(variants)):
                course = course + variants[i][0]
                distance = distance + variants[i][1]
            course1  = course /len(variants)
            distance1 = distance /len(variants)
            if very_Fast:
                if distance1 !=0:
                    self.local.localisation_Complete()
                    dist = distance1 / 1000
                    course_global_rad = course1 + self.glob.pf_coord[2]
                    self.glob.ball_coord = [dist * math.cos(course_global_rad) + self.glob.pf_coord[0],
                                            dist * math.sin(course_global_rad) + self.glob.pf_coord[1]]
                    return(a, course1, dist, [0, 0])
            self.neck_pan =int( - course1/ self.TIK2RAD)
            D = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK']- self.params['DIAMETER_OF_BALL']/2
            E = (2*distance1*D - math.sqrt(4*distance1**2*D**2 - 4*(distance1**2-self.params['HEIGHT_OF_NECK']**2)*(D**2 -self.params['HEIGHT_OF_NECK']**2)))/(2*(D**2-self.params['HEIGHT_OF_NECK']**2))
            alpha = math.atan(E)
            alpha_d = math.pi/2 - alpha
            self.neck_tilt = int((-alpha_d)/self.TIK2RAD + self.neck_calibr)
            #uprint('self.neck_pan =', self.neck_pan, 'self.neck_tilt =', self.neck_tilt)
            if self.glob.SIMULATION == 5:
                self.head_Return(self.neck_pan, self.neck_tilt)
            else:
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                             self.jointHandle[21] , self.neck_pan * self.TIK2RAD * self.ACTIVESERVOS[21][3], self.sim.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                             self.jointHandle[22] , self.neck_tilt * self.TIK2RAD * self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(16):
                    self.sim_simxSynchronousTrigger(self.clientID)
            self.refresh_Orientation()
            a, course, dist, speed = self.vision.detect_Ball_Speed()
            #if with_Localization: self.local.localisation_Complete()
            self.local.localisation_Complete()
            if a == True:
                #if with_Localization: self.local.localisation_Complete()
                course_global_rad = course + self.glob.pf_coord[2]
                self.glob.ball_coord = [dist*math.cos(course_global_rad)+ self.glob.pf_coord[0],
                                         dist*math.sin(course_global_rad)+ self.glob.pf_coord[1]]
                return(a, course, dist, speed)
            else:
                if distance1 !=0:
                    #if with_Localization: self.local.localisation_Complete()
                    dist = distance1 / 1000
                    course_global_rad = course1 + self.glob.pf_coord[2]
                    self.glob.ball_coord = [dist * math.cos(course_global_rad) + self.glob.pf_coord[0],
                                            dist * math.sin(course_global_rad) + self.glob.pf_coord[1]]
                    return(a, course1, dist, [0, 0])
        #if with_Localization: self.local.localisation_Complete()
        self.local.localisation_Complete()
        return False, 0, 0, [0, 0]

    def watch_Ball_In_Pose(self, penalty_Goalkeeper = False):
        self.local.correct_yaw_in_pf()
        if self.robot_In_0_Pose == False:
            if self.glob.SIMULATION == 5:
                self.play_Soft_Motion_Slot(name = 'Initial_Pose')
            else:
                self.simulateMotion(name = 'Initial_Pose')
            self.robot_In_0_Pose = True
        # U19 - Шея поворот
        # U20 - Шея Наклон
        c = self.neck_play_pose
        head_pose = [(-2667,c), (-1333, c) , ( 0, c) , (1333, c) , (2667,c),
                     (-2667, c-700),(-1333, c-700), (0, c-700), (1333,c-700),(2667, c-700),
                    (-2667, c-1400), (-1333, c-1400), ( 0, c-1400), (1333, c-1400), (2667, c-1400)]
        head_pose_seq = [2,7,12,11,6,8,13,14,9,4,3,10,5,0,1,2]
        if penalty_Goalkeeper: head_pose_seq = [2,7,12,11,6,8,13]
        for i in range(len(head_pose_seq)):
            if i != 0:
                x = head_pose[head_pose_seq[i]]
                self.neck_pan = x[0]
                self.neck_tilt = x[1]
            if not self.falling_Test() == 0:
                self.local.quality =0
                if self.falling_Flag == 3: uprint('STOP!')
                else: uprint('FALLING!!!', self.falling_Flag)
                return False, 0, 0, [0, 0]
            if self.glob.SIMULATION == 5:
                self.head_Return(self.neck_pan, self.neck_tilt)
            else:
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[21] , self.neck_pan * self.TIK2RAD * self.ACTIVESERVOS[21][3], self.sim.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                         self.jointHandle[22] , self.neck_tilt * self.TIK2RAD * self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(20):
                    self.sim_simxSynchronousTrigger(self.clientID)
            self.refresh_Orientation()
            a, course, dist, speed = self.vision.detect_Ball_Speed()
            if a == True or (a== False and dist !=0): break
        if a == True or (a== False and dist !=0):
            course_global_rad = course + self.glob.pf_coord[2]
            self.glob.ball_coord = [dist*math.cos(course_global_rad)+ self.glob.pf_coord[0],
                                        dist*math.sin(course_global_rad)+ self.glob.pf_coord[1]]
            if len(self.glob.obstacles) == 0: self.glob.obstacles = [[0,0,0]]
            self.glob.obstacles[0] = [self.glob.ball_coord[0], self.glob.ball_coord[1], 0.15]
            distance = dist *1000
            self.neck_pan =int( - course/ self.TIK2RAD)
            D = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK']- self.params['DIAMETER_OF_BALL']/2
            E = (2*distance*D - math.sqrt(4*distance**2*D**2 - 4*(distance**2-self.params['HEIGHT_OF_NECK']**2)*(D**2 -self.params['HEIGHT_OF_NECK']**2)))/(2*(D**2-self.params['HEIGHT_OF_NECK']**2))
            alpha = math.atan(E)
            alpha_d = math.pi/2 - alpha
            self.neck_tilt = int((-alpha_d)/self.TIK2RAD + self.neck_calibr)
            return(a, course, dist, speed)
        return False, 0, 0, [0, 0]

    def get_course_and_distance_to_post(self, blob_cx, blob_y_plus_h):   # returns course in degrees and distance in mm
        c = self.neck_calibr
        if self.glob.SIMULATION == 2:
            z,a = self.kondo.getUserParameter(19)
            z,b = self.kondo.getUserParameter(20)
            a = self.neck_pan
            b = self.neck_tilt
            # U19 - Шея поворот
            # U20 - Шея Наклон
        else:
            returnCode, position21= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[21], self.sim.simx_opmode_blocking)
            a = position21*self.ACTIVESERVOS[21][3]*1698        # Шея поворот
            returnCode, position22= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[22], self.sim.simx_opmode_blocking)
            b = position22*self.ACTIVESERVOS[22][3]*1698        # Шея Наклон
        #self.refresh_Orientation()
        head_to_horizon_angle = self.euler_angle['pitch'] - self.head_pitch_with_horizontal_camera
        #y = head_to_horizon_angle - math.radians((self.params['CAMERA_VERTICAL_RESOLUTION'] / 2 - blob_y_plus_h)* self.params['APERTURE_PER_PIXEL_VERTICAL'])
        y = head_to_horizon_angle - math.atan((self.params['CAMERA_VERTICAL_RESOLUTION'] / 2 - blob_y_plus_h) / self.focal_distance_in_pixels)
        vision_height = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK'] + self.params['HEIGHT_OF_NECK']*math.cos(head_to_horizon_angle)
        if y == 0:
            distance_in_mm1 = vision_dist = 4000
        else:
            distance_in_mm1 = vision_height / math.tan(y) + self.params['HEIGHT_OF_NECK']*math.sin(head_to_horizon_angle)
            vision_dist = vision_height/math.sin(y)
        #vision_shift = vision_dist * math.tan(math.radians((self.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2 - blob_cx) * self.params['APERTURE_PER_PIXEL']))
        vision_shift = vision_dist * (self.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2 - blob_cx) / self.focal_distance_in_pixels
        distance_in_mm2 = math.sqrt(distance_in_mm1**2 + vision_shift**2)
        #course1 = self.euler_angle['yaw'] + math.atan(vision_shift/distance_in_mm1)
        #course = self.euler_angle['yaw'] + math.radians((self.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2 - blob_cx) * self.params['APERTURE_PER_PIXEL'])
        #course = self.euler_angle['yaw'] + math.atan((self.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2 - blob_cx) / self.focal_distance_in_pixels )
        course = self.euler_angle['yaw'] + math.atan( vision_shift / distance_in_mm1)
        #uprint('distance_in_mm1 = ', distance_in_mm1, 'distance_in_mm2 = ', distance_in_mm2 )
        #uprint('course =', course, 'course1 = ', course1)
        return course, distance_in_mm2/1000

    def get_cooord_of_point(self, point_x, point_y):   # takes x,y of point in QVGA frame and returns x,y of point in m in local coordinates of robot
        try:
            #self.refresh_Orientation()
            head_to_horizon_angle = self.euler_angle['pitch'] - self.head_pitch_with_horizontal_camera
            #vision_line_angle = head_to_horizon_angle + math.radians((point_y - self.params['CAMERA_VERTICAL_RESOLUTION'] / 2) * self.params['APERTURE_PER_PIXEL_VERTICAL'])     # 0.18925
            vision_line_angle = head_to_horizon_angle + math.atan((point_y - self.params['CAMERA_VERTICAL_RESOLUTION'] / 2) / self.focal_distance_in_pixels)
            vision_height = self.params['HEIGHT_OF_CAMERA'] - self.params['HEIGHT_OF_NECK'] + self.params['HEIGHT_OF_NECK']*math.cos(head_to_horizon_angle)
            distance_in_mm1 = vision_height / math.tan(vision_line_angle) + self.params['HEIGHT_OF_NECK']*math.sin(head_to_horizon_angle)
            vision_dist = vision_height/math.sin(vision_line_angle)
            vision_shift = vision_dist * (self.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2 - point_x)  / self.focal_distance_in_pixels
            distance_in_mm2 = math.sqrt(distance_in_mm1**2 + vision_shift**2)
            course =  math.atan(vision_shift/distance_in_mm1) - self.neck_pan * self.TIK2RAD
            x = distance_in_mm2/1000 * math.cos(course)
            y = distance_in_mm2/1000 * math.sin(course)
        except Exception: return False, 0, 0
        return True, x, y

    def turn_To_Course(self, course, accurate = False, one_Off_Motion = True):
        stepLength = 0
        sideLength = 0
        rotation = 0
        cycleNumber = 1
        cycle = 0
        target = course # + self.direction_To_Attack
        #if one_Off_Motion: old_neck_pan, old_neck_tilt = self.head_Up()
        self.refresh_Orientation()
        rotation1 = target - self.body_euler_angle['yaw']
        rotation1 = self.norm_yaw(rotation1)
        if rotation1 <= 0:
            self.first_Leg_Is_Right_Leg = True
            invert = 1
        else:
            self.first_Leg_Is_Right_Leg = False
            invert = -1
        if abs(rotation1)> 0.035 or accurate:
            cycleNumber = int(math.floor(abs(rotation1)/0.23))+1       # rotation yield 0.23 with rotation order 0.21
            if one_Off_Motion: 
                self.walk_Initial_Pose()
                after_cycle = 0
            else: after_cycle = 1
            for cycle in range (cycleNumber):
                self.refresh_Orientation()
                rotation1 = target - self.body_euler_angle['yaw']
                rotation1 = self.norm_yaw(rotation1)
                if abs(rotation1)< 0.035 and not accurate: break
                if abs(rotation1)< 0.01: break
                rotation = rotation1/(cycleNumber - cycle)
                rotation = self.normalize_rotation(rotation)
                #uprint('self.euler_angle[0]=', self.euler_angle[0],'rotation =', rotation )
                self.walk_Cycle(stepLength, sideLength,invert*rotation,cycle,cycleNumber + after_cycle)
            if one_Off_Motion: self.walk_Final_Pose()
        self.refresh_Orientation()
        self.local.coord_odometry[2] = self.body_euler_angle['yaw']
        self.local.coord_shift = [0,0,0]
        self.local.coordinate_record(odometry = True, shift = True)
        self.local.refresh_odometry()
        #if one_Off_Motion: self.head_Return(old_neck_pan, old_neck_tilt)
        self.first_Leg_Is_Right_Leg = True

    def head_Up(self):
        old_neck_pan = self.neck_pan
        old_neck_tilt = self.neck_tilt
        self.neck_pan = 0
        self.neck_tilt = self.neck_play_pose
        if self.glob.SIMULATION == 5:
            pan_pos = int(self.neck_pan * self.ACTIVESERVOS[21][2]) + 7500
            tilt_pos = int(self.neck_tilt * self.ACTIVESERVOS[22][2]) + 7500
            servoDatas = [self.Roki.Rcb4.ServoData(), self.Roki.Rcb4.ServoData()]
            servoDatas[0].Id, servoDatas[0].Sio, servoDatas[0].Data = self.ACTIVESERVOS[21][0], self.ACTIVESERVOS[21][1], pan_pos
            servoDatas[1].Id, servoDatas[1].Sio, servoDatas[1].Data = self.ACTIVESERVOS[22][0], self.ACTIVESERVOS[22][1], tilt_pos
            a=self.rcb.setServoPos (servoDatas, 10)
            time.sleep(0.2)
        else:
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[21] , 0, self.sim.simx_opmode_oneshot)   # Шея поворот
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[22] , self.neck_play_pose*0.000589*self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot)  # Шея Наклон
            for i in range(16):
                if self.glob.SIMULATION != 0:
                    self.sim_simxSynchronousTrigger(self.clientID)
        self.refresh_Orientation()
        return old_neck_pan, old_neck_tilt

    def head_Return(self, old_neck_pan, old_neck_tilt):
        if self.glob.SIMULATION == 5:
            pan_pos = int(old_neck_pan * self.ACTIVESERVOS[21][2]) + 7500
            tilt_pos = int(old_neck_tilt * self.ACTIVESERVOS[22][2]) + 7500
            servoDatas = [self.Roki.Rcb4.ServoData(), self.Roki.Rcb4.ServoData()]
            servoDatas[0].Id, servoDatas[0].Sio, servoDatas[0].Data = self.ACTIVESERVOS[21][0], self.ACTIVESERVOS[21][1], pan_pos
            servoDatas[1].Id, servoDatas[1].Sio, servoDatas[1].Data = self.ACTIVESERVOS[22][0], self.ACTIVESERVOS[22][1], tilt_pos
            a=self.rcb.setServoPos (servoDatas, 10)
            time.sleep(0.2)
        else:
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[21] , old_neck_pan*0.000589*self.ACTIVESERVOS[21][3], self.sim.simx_opmode_oneshot)   # Шея поворот
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[22] , old_neck_tilt*0.000589*self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot)  # Шея Наклон
            for i in range(16):
                if self.glob.SIMULATION != 0:
                    self.sim_simxSynchronousTrigger(self.clientID)
        self.refresh_Orientation()

    def normalize_rotation(self, yaw):
        if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
        if yaw > math.pi : yaw -= (2 * math.pi)
        if yaw < -math.pi : yaw += (2 * math.pi)
        if yaw > 0.3 : yaw = 0.3
        if yaw < -0.3 : yaw = -0.3
        return yaw

    def near_distance_omni_motion(self, dist_mm, napravl, one_Off_Motion = True):
        #if one_Off_Motion: old_neck_pan, old_neck_tilt = self.head_Up()
        dist = dist_mm/1000
        self.refresh_Orientation()
        initial_direction = self.imu_body_yaw()
        print('initial_direction', initial_direction)
        n = int(math.floor((dist_mm*math.cos(napravl)-self.first_step_yield)/self.cycle_step_yield)+1)+1         #calculating the number of potential full steps forward
        displacement = dist_mm*math.sin(napravl)
        first_Leg_Is_Right_Leg_Old = self.first_Leg_Is_Right_Leg
        if displacement > 0:
            self.first_Leg_Is_Right_Leg = False
            side_step_yield = self.side_step_left_yield
        else:
            side_step_yield = self.side_step_right_yield
        if self.first_Leg_Is_Right_Leg: invert = 1
        else: invert = -1
        m = int(math.floor(abs(displacement)/side_step_yield)+1)
        if n < m : n = m
        stepLength = dist_mm*math.cos(napravl)/(self.first_step_yield*1.25+self.cycle_step_yield*(n-1)+ self.cycle_step_yield*0.75)*64
        number_Of_Cycles = n+2
        sideLength = abs(displacement) /number_Of_Cycles*20/side_step_yield
        if stepLength > 15 and number_Of_Cycles > 4: 
            deceleration = True
            number_Of_Cycles += 1
        else: deceleration = False
        self.local.correct_yaw_in_pf()
        if one_Off_Motion: 
            self.walk_Initial_Pose()
            after_cycle = 0
        else: after_cycle = 1
        for cycle in range(number_Of_Cycles):
            self.refresh_Orientation()
            rotation = initial_direction - self.imu_body_yaw() * 1
            rotation = self.normalize_rotation(rotation)
            stepLength1 = stepLength
            if cycle == 0: stepLength1 = stepLength/4
            if cycle == 1: stepLength1 = stepLength/2
            if deceleration:
                if cycle == number_Of_Cycles - 1: stepLength1 = stepLength / 3
                if cycle == number_Of_Cycles - 2: stepLength1 = stepLength * 2 / 3
            self.walk_Cycle(stepLength1, sideLength, invert*rotation,cycle,number_Of_Cycles + after_cycle)
        if one_Off_Motion: self.walk_Final_Pose()
        self.first_Leg_Is_Right_Leg = first_Leg_Is_Right_Leg_Old
        self.local.coord_odometry[0] += dist * math.cos(napravl)
        self.local.coord_odometry[1] += dist * math.sin(napravl)
        #self.local.coordinate_record(odometry = True)
        #if one_Off_Motion: self.head_Return(old_neck_pan, old_neck_tilt)

    def near_distance_ball_approach_and_kick(self, kick_direction, strong_kick = False, small_kick = False, very_Fast = False ):
        offset_of_ball = self.params['KICK_OFFSET_OF_BALL']  # self.d10 # module of local robot Y coordinate of ball im mm before kick 
        a, napravl, dist, speed = self.seek_Ball_In_Pose(fast_Reaction_On = True, very_Fast = very_Fast,
                                   first_look_point = self.glob.ball_coord, with_Localization = False)
        dist_mm = dist *1000
        if a==False or self.falling_Flag != 0: return False
        if dist > 0.9 or a == False: return False
        if  0.02 < abs(dist * math.cos(napravl)) < 0.06 and dist * math.sin(napravl) < 0.03:
            #old_neck_pan, old_neck_tilt = self.head_Up()
            if napravl > 0: self.kick(first_Leg_Is_Right_Leg=False)
            else: self.kick(first_Leg_Is_Right_Leg=True)
            #self.head_Return(old_neck_pan, old_neck_tilt)
        if abs(napravl) > 1 :
            direction = math.copysign(2.55, napravl)
            self.near_distance_omni_motion( 180 , direction)
        else:
            forth_dist = dist_mm*math.cos(napravl) 
            n = int(math.ceil((forth_dist - self.params['KICK_ADJUSTMENT_DISTANCE_1']
                                -self.first_step_yield)/self.cycle_step_yield)+1)         #calculating the number of potential full steps forward
            displacement = dist_mm*math.sin(napravl)- math.copysign(offset_of_ball, napravl)
            if displacement > 0:
                invert = -1
                self.first_Leg_Is_Right_Leg = False
                side_step_yield = self.side_step_left_yield
            else:
                invert = 1
                side_step_yield = self.side_step_right_yield
            m = int(math.ceil(abs(displacement)/side_step_yield))       # potential side steps number
            if n < m : n = m
            n += 2                                                      # final steps number
            stepLength = (dist_mm*math.cos(napravl)-
                          self.params['KICK_ADJUSTMENT_DISTANCE_1'])/(self.first_step_yield
                          + self.cycle_step_yield * n) * 64
            number_Of_Cycles = n + 1
            if napravl > 0:
                kick_by_Right = False
            else:
                kick_by_Right = True
            sideLength = abs(displacement)/number_Of_Cycles*20/side_step_yield
            #old_neck_pan, old_neck_tilt = self.head_Up()
            self.local.correct_yaw_in_pf()
            init_yaw = kick_direction #= self.imu_body_yaw()
            #init_yaw = self.imu_body_yaw()
            stepLengthResidue = 0
            sideLengthResidue = 0
            stepLengthResidue_accumulated = 0
            #sideLengthResidue_accumulated = 0
            self.walk_Initial_Pose()
            cycle = 0
            while (cycle < number_Of_Cycles):
                rotation = (kick_direction - self.imu_body_yaw()) * 1
                rotation = self.normalize_rotation(rotation)
                stepLength1 = stepLength
                sideLength1 = sideLength
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
                stepLength1 += stepLengthResidue
                #sideLength1 += sideLengthResidue
                self.walk_Cycle(stepLength1, sideLength1, invert*rotation,cycle,number_Of_Cycles)
                self.refresh_Orientation()
                delta_yaw = self.norm_yaw(self.imu_body_yaw() - init_yaw)
                stepLengthResidue = stepLength1 * (1 - math.cos(delta_yaw)) - sideLength1 * math.sin(delta_yaw) * invert
                stepLengthResidue_accumulated += stepLengthResidue
                #sideLengthResidue = sideLength1 * (1 - math.cos(delta_yaw)) + stepLength1 * math.sin(delta_yaw) * invert
                #sideLengthResidue_accumulated += sideLengthResidue
                cycle += 1
            self.walk_Final_Pose()
            print('stepLengthResidue_accumulated =', stepLengthResidue_accumulated)
            self.first_Leg_Is_Right_Leg = True
            #self.stepHeight = 20
            #stop_detecting_Ball = threading.Event()
            #camera_thread = threading.Thread(target = self.glob.vision.detect_Ball_in_Stream, args=(stop_detecting_Ball,))
            #camera_thread.setDaemon(True)
            #camera_thread.start()
            result, kick_by_Right = self.verify_ball_position(kick_by_Right, kick_direction)
            #stop_detecting_Ball.set()
            self.first_Leg_Is_Right_Leg = True
            #self.stepHeight = 32
            #self.head_Up()
            if strong_kick == True:
                if kick_by_Right == True:
                    self.play_Motion_Slot(name = 'Soccer_Kick_Forward_Right_Leg')
                else:
                    self.play_Motion_Slot(name = 'Soccer_Kick_Forward_Left_Leg')
            else:
                self.kick( first_Leg_Is_Right_Leg=kick_by_Right, small = small_kick)
            self.local.coord_odometry[0] += dist * math.cos(napravl)
            self.local.coord_odometry[1] += dist * math.sin(napravl)
            self.pause_in_ms(1000)
            #self.head_Return(old_neck_pan, old_neck_tilt)
        return True

    def verify_ball_position(self, kick_by_Right, kick_direction):
        def moving_direction(kick_by_Right):
            if self.glob.ball_distance <= 0: return False , 0, 0, kick_by_Right
            #ball_x = self.glob.ball_distance * math.cos(self.glob.ball_course) * 1000
            #ball_y = self.glob.ball_distance * math.sin(self.glob.ball_course) * 1000
            #dx = self.glob.ball_coord[0] - self.glob.pf_coord[0]
            #dy = self.glob.ball_coord[1] - self.glob.pf_coord[1]
            #ball_x = (dx * math.cos(self.glob.pf_coord[2]) + dy * math.sin(self.glob.pf_coord[2])) * 1000
            #ball_y = (dy * math.cos(self.glob.pf_coord[2]) - dx * math.sin(self.glob.pf_coord[2])) * 1000
            r_x, r_y, r_yaw = self.local.coord_odometry
            b_x, b_y = self.local.ball_odometry
            dx = b_x - r_x
            dy = b_y - r_y
            ball_x = (dx * cos(r_yaw) + dy * sin(r_yaw)) * 1000
            ball_y = (dy * cos(r_yaw) - dx * sin(r_yaw)) * 1000
            if  ball_y > 20: kick_by_Right = False
            if ball_y < -20: kick_by_Right = True
            if kick_by_Right: side_motion = ball_y + self.params["KICK_OFFSET_OF_BALL"]
            else: side_motion = ball_y - self.params["KICK_OFFSET_OF_BALL"]
            front_motion = ball_x - self.params["KICK_ADJUSTMENT_DISTANCE_2"]
            print('front_motion: ', round(front_motion), 'side_motion: ', round(side_motion))
            return True, front_motion, side_motion, kick_by_Right

        front_motion_tolerance = 30
        side_motion_tolerance = 20
        result, front_motion, side_motion, kick_by_Right = moving_direction(kick_by_Right)
        if not result:
            #self.glob.camera_ON = False
            return result, kick_by_Right
        elif result and front_motion <= front_motion_tolerance and abs(side_motion) < side_motion_tolerance: 
            #self.glob.camera_ON = False
            return result, kick_by_Right
        self.first_Leg_Is_Right_Leg = True
        motion_to_right = (side_motion < 0)
        #if side_motion <= 0:
        #    self.first_Leg_Is_Right_Leg = True
        #    invert = 1
        #else:
        #    self.first_Leg_Is_Right_Leg = False
        #    invert = -1
        self.gaitHeight= 180
        self.walk_Initial_Pose()
        number_Of_Cycles = 50
        for cycle in range(number_Of_Cycles):
            if (motion_to_right and (side_motion >= 0)) or ((not motion_to_right) and (side_motion < 0)) : self.walk_Restart()
            self.refresh_Orientation()
            rotation = kick_direction - self.body_euler_angle['yaw'] * 1.1
            rotation = self.normalize_rotation(rotation)
            if front_motion > front_motion_tolerance: stepLength = 30
            elif front_motion < -front_motion_tolerance: stepLength = -30
            else: stepLength = front_motion * 64 / self.cycle_step_yield
            #print('side_motion =', side_motion)
            if side_motion  < -side_motion_tolerance : sideLength = 20 
            elif side_motion > side_motion_tolerance : sideLength = -20
            elif side_motion > 0: sideLength = -side_motion * 20 / self.glob.side_step_left_yield
            else: sideLength = -side_motion * 20 / self.side_step_right_yield 
            #self.walk_Cycle(stepLength, sideLength,  rotation, cycle, number_Of_Cycles)
            if self.glob.ball_distance > 0.7 or (result and front_motion <= front_motion_tolerance and abs(side_motion) < side_motion_tolerance) or not result:
                self.walk_Cycle(stepLength, sideLength,  rotation, cycle, cycle + 1)
                break
            else:
                self.walk_Cycle(stepLength, sideLength, rotation, cycle, number_Of_Cycles)
            result, front_motion, side_motion, kick_by_Right =  moving_direction(kick_by_Right)
        self.walk_Final_Pose()
        return result, kick_by_Right

    def near_distance_ball_approach_and_kick_streaming(self, kick_direction, small_kick = False):
        print('near_distance_ball_approach_and_kick_streaming')
        offset_of_ball = self.params['KICK_OFFSET_OF_BALL']  # self.d10 # module of local robot Y coordinate of ball im mm before kick 
        if self.glob.ball_distance > 0.9 or self.glob.robot_see_ball <= 0: return False
        dist = self.glob.ball_distance
        napravl = self.glob.ball_course
        dist_mm = self.glob.ball_distance *1000
        if  0.02 < abs(dist * math.cos(napravl)) < 0.06 and dist * math.sin(napravl) < 0.03:
            if napravl > 0: self.kick(first_Leg_Is_Right_Leg=False)
            else: self.kick(first_Leg_Is_Right_Leg=True)
        if abs(napravl) > 1 :
            direction = math.copysign(2.55, napravl)
            self.near_distance_omni_motion( 180 , direction)
        else:
            if napravl > 0:
                kick_by_Right = False
            else:
                kick_by_Right = True
            self.first_Leg_Is_Right_Leg = True
            result, kick_by_Right = self.verify_ball_position(kick_by_Right, kick_direction)
            self.first_Leg_Is_Right_Leg = True
            if self.glob.ball_distance < 0.2:
                self.kick( first_Leg_Is_Right_Leg=kick_by_Right, small = small_kick)
                #if small_kick:
                #    self.kick( first_Leg_Is_Right_Leg=kick_by_Right, small = small_kick)
                #else:
                #    if kick_by_Right:
                #        self.play_Soft_Motion_Slot(name ='Kick_Right_v2')
                #    else:
                #        self.play_Soft_Motion_Slot(name ='Kick_Left_v2')
                self.pause_in_ms(1000)
        return True

    def kick_off_ride(self):
        print('kick_off_ride')
        angle = 30
        distance = 0.55
        sideLength = 0
        acceleration = False
        deceleration = False
        invert = round(random.random(),0)*2 - 1
        self.walk_Initial_Pose()
        delta_yaw = invert * math.radians(angle) #0.52 #0.79
        number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
        delta_yaw_step = delta_yaw / number_Of_Cycles
        stepLength = 0
        self.head_Return(0, self.neck_play_pose)
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            self.refresh_Orientation()
            rotation = 0 + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
            rotation = self.normalize_rotation(rotation)
            self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles+1)
        delta_yaw = -2 * math.radians(angle)  #-1.04 # -1.58
        number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
        delta_yaw_step = delta_yaw / number_Of_Cycles
        R = distance / 2 / math.sin(math.radians(angle))   #0.45 # 0.32
        stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
        acceleration = True
        #number_Of_Cycles += 1
        deceleration = True
        number_Of_Cycles -= 2
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if acceleration:
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
            if deceleration:
                if cycle == number_Of_Cycles - 1: stepLength1 = stepLength - (stepLength - 30) * 2 / 3
                if cycle == number_Of_Cycles - 2: stepLength1 = stepLength - (stepLength - 30) / 3
            self.refresh_Orientation()
            rotation = (math.radians(angle) + delta_yaw_step * (cycle))  * invert - self.imu_body_yaw()
            rotation = self.normalize_rotation(rotation)
            self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
        stepLength_old = stepLength
        acceleration = False
        deceleration = False
        L = 0.6
        number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
        #stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
        stepLength = 30
        if stepLength - stepLength_old > 22 :
            acceleration = True
            number_Of_Cycles += 1
        deceleration = True
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if acceleration:
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
            if deceleration:
                if cycle == number_Of_Cycles - 1: stepLength1 = stepLength / 3
                if cycle == number_Of_Cycles - 2: stepLength1 = stepLength * 2 / 3
                #print('steplength = ', stepLength1)
            self.refresh_Orientation()
            if self.glob.robot_see_ball > 0: rotation = self.glob.ball_course / 5
            else: rotation = -math.radians(angle) * invert - self.imu_body_yaw()
            rotation = self.normalize_rotation(rotation)
            self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
        stepLength_old = stepLength
        acceleration = False
        deceleration = False
        self.walk_Final_Pose()
        self.first_Leg_Is_Right_Leg = True

    def far_distance_straight_approach(self, ball_coord, target_yaw, gap = 0.2, stop_Over = False):
        print('far_distance_straight_approach: initial arc')
        start_yaw = self.imu_body_yaw()   #self.glob.pf_coord[2]  #self.imu_body_yaw()
        #old_neck_pan, old_neck_tilt = self.head_Up()
        #self.local.correct_yaw_in_pf()
        sideLength = 0
        acceleration = False
        deceleration = False
        self.gaitHeight= 180
        self.walk_Initial_Pose()
# initial arc
        dest_yaw = target_yaw
        delta_yaw = self.norm_yaw(dest_yaw - start_yaw)
        number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
        if number_Of_Cycles == 0: number_Of_Cycles = 1
        delta_yaw_step = delta_yaw / number_Of_Cycles
        stepLength = 0
        for cycle in range(number_Of_Cycles):
            self.refresh_Orientation()
            rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
            rotation = self.normalize_rotation(rotation)
            self.walk_Cycle(stepLength, sideLength, rotation, cycle, number_Of_Cycles+1)
        print('target yaw', target_yaw)
        #self.turn_To_Course(target_yaw, accurate = True)
        #self.walk_Initial_Pose()
        self.refresh_Orientation()
        print('imu_body_yaw', self.imu_body_yaw())
        acceleration = False
        deceleration = False
# straight segment 
        print('far_distance_straight_approach: straight segment')
        L = math.sqrt((ball_coord[0]-self.local.coord_odometry[0])**2 + (ball_coord[1]-self.local.coord_odometry[1])**2) - gap
        number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
        stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
        if stepLength > 22 :
            acceleration = True
            number_Of_Cycles += 1
        if not stop_Over:
            if stepLength  > 15:
                deceleration = True
                number_Of_Cycles += 1
            for cycle in range(number_Of_Cycles):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                if deceleration:
                    if cycle == number_Of_Cycles - 1: stepLength1 = stepLength - (stepLength ) * 2 / 3
                    if cycle == number_Of_Cycles - 2: stepLength1 = stepLength - (stepLength ) / 3
                self.refresh_Orientation()
                if self.glob.robot_see_ball > 0:
                    rotation = self.glob.ball_course / 10
                else:
                    rotation = dest_yaw - self.imu_body_yaw()
                print('self.glob.ball_course:', self.glob.ball_course)
                rotation = self.normalize_rotation(rotation)
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+1)
        else:
            print('with stop_over')
            number_Of_Cycles = math.ceil(number_Of_Cycles/2)
            if stepLength > 15:
                    deceleration = True
                    number_Of_Cycles += 1
            else: deceleration = False
            for cycle in range(1, number_Of_Cycles + 1, 1):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                self.refresh_Orientation()
                rotation = dest_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                if deceleration:
                    if cycle == number_Of_Cycles: stepLength1 = stepLength / 3
                    if cycle == number_Of_Cycles - 1: stepLength1 = stepLength * 2 / 3
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles + 1)
        self.walk_Final_Pose()
        #self.head_Return(old_neck_pan, old_neck_tilt)
        return True

    def far_distance_plan_approach(self, ball_coord, target_yaw, stop_Over = False):
        dest = []
        centers = []
        price = 1000
        for i in range(5):
                for j in range(2):
                    target_x = ball_coord[0] - (0.21 + j * 0.05) * math.cos(target_yaw - 0.8 + i * 0.4)
                    target_y = ball_coord[1] - (0.21 + j * 0.05) * math.sin(target_yaw - 0.8 + i * 0.4)
                    target_coord = [target_x, target_y, target_yaw]
                    dest1, centers1, number_Of_Cycles = self.p.path_calc_optimum(self.glob.pf_coord, target_coord)
                    if i != 2: number_Of_Cycles += 50
                    if number_Of_Cycles <= price:
                        dest = dest1
                        centers = centers1
                        price = number_Of_Cycles
        if stop_Over: price += 100
        walking_distances = (len(centers)-1) * 2 + 1
#  steplength list forming
        steplength_list = []
        for arc_number in range(len(centers)):
            if arc_number == 0:
                start_yaw = self.glob.pf_coord[2]
            else:
                start_yaw = dest_yaw
            if arc_number < len(centers) - 1:
                dest_yaw = self.p.coord2yaw(dest[arc_number*2 + 1][0] - dest[arc_number*2][0], dest[arc_number*2 + 1][1] - dest[arc_number*2][1] )
            else: 
                dest_yaw = target_yaw
            x1, y1, x2, y2, cx, cy, R, CW = centers[arc_number]
            delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
            number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
            delta_yaw_step = delta_yaw / number_Of_Cycles
            stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
            steplength_list.append(stepLength)
            if arc_number < len(centers) - 1:
                L = math.sqrt((dest[arc_number*2 + 1][0] - dest[arc_number*2][0])**2 + (dest[arc_number*2 + 1][1] - dest[arc_number*2][1])**2)
                number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
                stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
                steplength_list.append(stepLength)
        start_yaw = self.glob.pf_coord[2]  #self.imu_body_yaw()
        if len(dest) == 0: return False
        #old_neck_pan, old_neck_tilt = self.head_Up()
        self.local.correct_yaw_in_pf()
        sideLength = 0
        stepLength_old = 0
        acceleration = False
        deceleration = False
        self.walk_Initial_Pose()
# initial arc
        dest_yaw = self.p.coord2yaw(dest[1][0] - dest[0][0], dest[1][1] - dest[0][1] )
        x1, y1, x2, y2, cx, cy, R, CW = centers[0]
        delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
        #print('delta_yaw:', delta_yaw, 'CW:', CW)
        number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
        delta_yaw_step = delta_yaw / number_Of_Cycles
        stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
        if stepLength - stepLength_old > 22 :
            acceleration = True
            number_Of_Cycles += 1
        if stepLength - steplength_list[1] > 15:
            deceleration = True
            number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if acceleration:
                if cycle == 0: stepLength1 = stepLength / 3
                if cycle == 1: stepLength1 = stepLength * 2 / 3
            if deceleration:
                if cycle == number_Of_Cycles - 1: stepLength1 = stepLength - (stepLength - steplength_list[1]) * 2 / 3
                if cycle == number_Of_Cycles - 2: stepLength1 = stepLength - (stepLength - steplength_list[1]) / 3
            self.refresh_Orientation()
            rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
            rotation = self.normalize_rotation(rotation)
            self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles+1)
        stepLength_old = stepLength
        acceleration = False
        deceleration = False
# 1-st straight segment 
        L = math.sqrt((dest[1][0] - dest[0][0])**2 + (dest[1][1] - dest[0][1])**2)
        number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
        stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
        if stepLength - stepLength_old > 22 :
            acceleration = True
            number_Of_Cycles += 1
        if not stop_Over:
            if stepLength - steplength_list[2] > 15:
                deceleration = True
                number_Of_Cycles += 1
                #print('deceleration = True')
            for cycle in range(number_Of_Cycles):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                if deceleration:
                    if cycle == number_Of_Cycles - 1: stepLength1 = stepLength - (stepLength - steplength_list[2]) * 2 / 3
                    if cycle == number_Of_Cycles - 2: stepLength1 = stepLength - (stepLength - steplength_list[2]) / 3
                    #print('steplength = ', stepLength1)
                self.refresh_Orientation()
                rotation = dest_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
            stepLength_old = stepLength
            acceleration = False
            deceleration = False
            for i in range(len(centers)-2):
                start_yaw = dest_yaw   #self.imu_body_yaw()
                dest_yaw = self.p.coord2yaw(dest[2*i+3][0] - dest[2*i+2][0], dest[2*i+3][1] - dest[2*i+2][1])
                x1, y1, x2, y2, cx, cy, R, CW = centers[i+1]
                delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
                #print('delta_yaw:', delta_yaw, 'CW:', CW)
                number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
                delta_yaw_step = delta_yaw / number_Of_Cycles
                stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    self.refresh_Orientation()
                    rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    if price < 100:
                        self.walk_Cycle(stepLength1, sideLength, rotation, cycle+1, number_Of_Cycles+2)
                    else:
                        self.walk_Cycle(stepLength1, sideLength, rotation, cycle+1, number_Of_Cycles+1)
                stepLength_old = stepLength
                acceleration = False
                if price >= 100: break
                L = math.sqrt((dest[2*i+3][0] - dest[2*i+2][0])**2 + (dest[2*i+3][1] - dest[2*i+2][1])**2)
                number_Of_Cycles = math.ceil(abs(L * 1000 / self.cycle_step_yield))
                stepLength = L * 1000 / number_Of_Cycles * 64 / self.cycle_step_yield
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    self.refresh_Orientation()
                    rotation = dest_yaw - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles+2)
                stepLength_old = stepLength
                acceleration = False
            if price < 100:
                start_yaw = dest_yaw   #self.imu_body_yaw()
                dest_yaw = target_yaw
                x1, y1, x2, y2, cx, cy, R, CW = centers[len(centers)-1]
                delta_yaw = self.p.delta_yaw(start_yaw, dest_yaw, CW)
                #print('delta_yaw:', delta_yaw, 'CW:', CW)
                number_Of_Cycles = math.ceil(abs(delta_yaw / 0.2))
                delta_yaw_step = delta_yaw / number_Of_Cycles
                stepLength = R * abs(delta_yaw_step) * 1000 * 64 / self.cycle_step_yield * 1.1
                if stepLength - stepLength_old > 22 :
                    acceleration = True
                    number_Of_Cycles += 1
                if stepLength > 15:
                    deceleration = True
                    number_Of_Cycles += 1
                for cycle in range(number_Of_Cycles):
                    stepLength1 = stepLength
                    if acceleration:
                        if cycle == 0: stepLength1 = stepLength / 3
                        if cycle == 1: stepLength1 = stepLength * 2 / 3
                    if deceleration:
                        if cycle == number_Of_Cycles - 1: stepLength1 = stepLength / 3
                        if cycle == number_Of_Cycles - 2: stepLength1 = stepLength * 2 / 3
                    self.refresh_Orientation()
                    rotation = start_yaw + delta_yaw_step * (cycle + 1) - self.imu_body_yaw()
                    rotation = self.normalize_rotation(rotation)
                    self.walk_Cycle(stepLength1, sideLength, rotation, cycle + 1, number_Of_Cycles + 1)
            # Adjustment of yaw position
            number_Of_Cycles = 4
            stepLength = 0
            for cycle in range(1, number_Of_Cycles+1, 1):
                self.refresh_Orientation()
                rotation = target_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                self.walk_Cycle(stepLength, sideLength, rotation, cycle, number_Of_Cycles+1)
        else:
            number_Of_Cycles = math.ceil(number_Of_Cycles/2)
            if stepLength > 15:
                    deceleration = True
                    number_Of_Cycles += 1
            else: deceleration = False
            for cycle in range(1, number_Of_Cycles + 1, 1):
                stepLength1 = stepLength
                if acceleration:
                    if cycle == 0: stepLength1 = stepLength / 3
                    if cycle == 1: stepLength1 = stepLength * 2 / 3
                self.refresh_Orientation()
                rotation = dest_yaw - self.imu_body_yaw()
                rotation = self.normalize_rotation(rotation)
                if deceleration:
                    if cycle == number_Of_Cycles: stepLength1 = stepLength / 3
                    if cycle == number_Of_Cycles - 1: stepLength1 = stepLength * 2 / 3
                self.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_Of_Cycles + 1)
        self.walk_Final_Pose()
        #self.head_Return(old_neck_pan, old_neck_tilt)
        return True

    def walk_Restart(self):
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        initPoses = int(self.fr1/2)
        self.ztr = - self.gaitHeight
        self.ztl = - self.gaitHeight
        if self.glob.SIMULATION == 5:
            self.rcb.motionPlay(3)
            self.wait_for_gueue_end()
            # while True:
            #     if self.stm_channel.mb.GetQueueInfo().NumRequests < 1: break
            #     time.sleep(0.02)
        for j in range (initPoses):
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            self.ytr = -self.d10 + (initPoses-(j+1))*self.amplitude/2 /initPoses
            self.ytl =  self.d10 + (initPoses-(j+1))*self.amplitude/2 /initPoses
            angles = self.computeAlphaForWalk(self.SIZES, self.LIMALPHA )
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                    if self.glob.SIMULATION == 1:
                        self.sim_simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    if self.model == 'Roki_2':
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
                        for i in range(joint_number):
                            if self.ACTIVESERVOS[i][0] == 8:
                                n = joint_number- 1 + self.ACTIVESERVOS[i][1]
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    if j == 0:
                        a=self.rcb.setServoPosAsync(servoDatas, 10, 0)
                        self.fill_queue_with_frames(9)
                        #time.sleep(0.25)
                    else:
                        a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            self.local.quality =0
            if self.falling_Flag == 3: uprint('STOP!')
            else: uprint('FALLING!!!', self.falling_Flag)
            return[]
        if self.glob.SIMULATION == 5:
            self.rcb.motionPlay(3)
            self.wait_for_gueue_end()
            # while True:
            #     if self.stm_channel.mb.GetQueueInfo().NumRequests < 1: break
            #     time.sleep(0.02)
        for j in range (initPoses):
            if self.glob.SIMULATION == 5: start1 = time.perf_counter()
            self.ytr = -self.d10 - j*self.amplitude/2 /initPoses
            self.ytl =  self.d10 - j*self.amplitude/2 /initPoses
            angles = self.computeAlphaForWalk(self.SIZES, self.LIMALPHA )
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    #self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                            returnCode = self.sim.simxSetJointPosition(self.clientID,
                                         self.jointHandle[i] , angles[i]*self.ACTIVESERVOS[i][3]+self.trims[i],
                                         self.sim.simx_opmode_oneshot)
                    #self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                    if self.glob.SIMULATION == 1:
                        self.sim_simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 5:
                    joint_number = len(angles)
                    if self.model == 'Roki_2':
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number + 2)]
                        for i in range(joint_number):
                            if self.ACTIVESERVOS[i][0] == 8:
                                n = joint_number- 1 + self.ACTIVESERVOS[i][1]
                                pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2]/2 + 7500)
                                servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                            else: pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    else:
                        servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(joint_number)]
                        for i in range(joint_number):
                            pos = int(angles[i]*1698 * self.ACTIVESERVOS[i][2] + 7500)
                            servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    if j == 0:
                        a=self.rcb.setServoPosAsync(servoDatas, 10, 0)
                        self.fill_queue_with_frames(9)
                        #time.sleep(0.25)
                    else:
                        a=self.rcb.setServoPosAsync(servoDatas, self.frames_per_cycle, 0)

        return True

    def control_Head_motion(self, stop_control_Head_motion):
        def head_turn(pan, tilt):
            if self.glob.SIMULATION == 5:
                pan_pos = int(pan * self.ACTIVESERVOS[21][2]) + 7500
                tilt_pos = int(tilt * self.ACTIVESERVOS[22][2]) + 7500
                servoDatas = [self.Roki.Rcb4.ServoData(), self.Roki.Rcb4.ServoData()]
                servoDatas[0].Id, servoDatas[0].Sio, servoDatas[0].Data = self.ACTIVESERVOS[21][0], self.ACTIVESERVOS[21][1], pan_pos
                servoDatas[1].Id, servoDatas[1].Sio, servoDatas[1].Data = self.ACTIVESERVOS[22][0], self.ACTIVESERVOS[22][1], tilt_pos
                a=self.rcb.setServoPos (servoDatas, 10)
            else:
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                            self.jointHandle[21] , pan * self.TIK2RAD * self.ACTIVESERVOS[21][3], self.sim.simx_opmode_oneshot)   # Шея поворот
                returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                            self.jointHandle[22] , tilt * self.TIK2RAD * self.ACTIVESERVOS[22][3], self.sim.simx_opmode_oneshot)  # Шея Наклон
                if self.glob.SIMULATION != 0:
                    self.sim_simxSynchronousTrigger(self.clientID)
            self.neck_pan = pan
            self.neck_tilt = tilt

        pan_increment = 340
        pan_limit = 1700
        pan_direction = 1
        while True:
            if stop_control_Head_motion.is_set(): break
            if self.falling_Flag == 0:
                #if need_nod:
                #    need_nod = False
                #    head_turn(self.neck_pan, self.neck_play_pose - 200)
                #    time.sleep(0.05)
                #    head_turn(self.neck_pan, self.neck_play_pose)
                if self.glob.robot_see_ball > 0:
                    head_pan_rad = -self.glob.ball_course
                    head_pan = int(head_pan_rad / self.TIK2RAD)
                    if head_pan > pan_limit: head_pan = pan_limit
                    if head_pan < -pan_limit: head_pan = -pan_limit
                    #head_turn(head_pan, self.neck_play_pose)
                    head_turn(0, self.neck_play_pose)
                    time.sleep(1)
                else:
                    if self.neck_pan >= pan_limit: pan_direction = -1
                    if self.neck_pan <= -pan_limit: pan_direction = 1
                    head_pan = self.neck_pan + pan_increment * pan_direction
                    #head_turn(head_pan, self.neck_play_pose)
                    head_turn(0, self.neck_play_pose)
                    time.sleep(0.1)



if __name__=="__main__":
    print('This is not main module!')


