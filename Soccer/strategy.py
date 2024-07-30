import sys
import os
import math
import json
import time
import Soccer.utility
import threading
import random
#from Soccer.Motion.class_stm_channel import STM_channel
#from Soccer.Vision.class_Vision_RPI import Vision_RPI
from multiprocessing import Process, Value
from roki2met import roki2met
import datetime
import numpy as np
from ctypes import c_bool


def coord2yaw(x, y):
    if x == 0:
        if y > 0 : yaw = math.pi/2
        else: yaw = -math.pi/2
    else: yaw = math.atan(y/x)
    if x < 0:
        if yaw > 0: yaw -= math.pi
        else: yaw += math.pi
    return yaw

class GoalKeeper:
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0

    def turn_Face_To_Guest(self):
        if self.local.coord_odometry[0] < 0:
            self.motion.turn_To_Course(0)
            self.direction_To_Guest = 0
            return
        elif self.local.coord_odometry[0] > 0.8 and abs(self.local.coord_odometry[1]) > 0.6:
            self.direction_To_Guest = math.atan(-self.local.coord_odometry[1]/(1.8-self.local.coord_odometry[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)
        elif self.local.coord_odometry[0] < 1.5 and abs(self.local.coord_odometry[1]) < 0.25:
            if (1.8-self.local.ball_odometry[0]) == 0: self.direction_To_Guest = 0
            else: self.direction_To_Guest = math.atan((0.4* (round(random.random(),0)*2 - 1)-
                                                       self.local.ball_odometry[1])/(1.8-self.local.ball_odometry[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)
            return
        else:
            self.direction_To_Guest = math.atan(-self.local.coord_odometry[1]/(2.8-self.local.coord_odometry[0]))
            self.motion.turn_To_Course(self.direction_To_Guest)

    def goto_Center(self):                      #Function for reterning to center position
        print('Function for reterning to center position')
        #if self.local.coordinate_trust_estimation() < 0.5: self.motion.localisation_Motion()
        player_X_m = self.local.coord_odometry[0]
        player_Y_m = self.local.coord_odometry[1]
        duty_position_x = - self.glob.landmarks['FIELD_LENGTH']/2 + 0.4
        distance_to_target = math.sqrt((duty_position_x -player_X_m)**2 + (0 - player_Y_m)**2 )
        if distance_to_target > 0.5 :
            target_in_front_of_duty_position = [duty_position_x + 0.15, 0]
            if distance_to_target > 1: stop_Over = True
            else: stop_Over = False
            self.motion.far_distance_plan_approach(target_in_front_of_duty_position, self.direction_To_Guest, stop_Over = stop_Over)
        else:
            if (duty_position_x -player_X_m)==0:
                alpha = math.copysign(math.pi/2, (0 - player_Y_m) )
            else:
                if (duty_position_x - player_X_m)> 0: alpha = math.atan((0 - player_Y_m)/(duty_position_x -player_X_m))
                else: alpha = math.atan((0 - player_Y_m)/(duty_position_x - player_X_m)) + math.pi
            napravl = alpha - self.motion.imu_body_yaw()
            dist_mm = distance_to_target * 1000
            self.motion.near_distance_omni_motion(dist_mm, napravl)
        self.turn_Face_To_Guest()

    def ball_Speed_Dangerous(self):
        pass
    def fall_to_Defence(self):
        print('fall to defence')
    def get_Up_from_defence(self):
        print('up from defence')
    def scenario_A1(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        print('The robot knock out the ball to the side of the enemy')
        for i in range(10):
            if dist > 0.5 :
                if dist > 1: stop_Over = True
                else: stop_Over = False
                self.motion.far_distance_plan_approach(self.local.ball_odometry, self.direction_To_Guest, stop_Over = stop_Over)
            self.turn_Face_To_Guest()
            success_Code = self.motion.near_distance_ball_approach_and_kick(self.direction_To_Guest)
            if success_Code == False and self.motion.falling_Flag != 0: return
            if success_Code == False : break
            #success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = False)
            dist = self.glob.ball_distance
            if dist > 1 : break
        target_course1 = self.local.coord_odometry[2] +math.pi
        self.motion.turn_To_Course(target_course1)
        self.goto_Center()

    def scenario_A2(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        print('The robot knock out the ball to the side of the enemy')
        self.scenario_A1( dist, napravl)

    def scenario_A3(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        print('The robot knock out the ball to the side of the enemy')
        self.scenario_A1( dist, napravl)

    def scenario_A4(self, dist, napravl):#The robot knock out the ball to the side of the enemy
        print('The robot knock out the ball to the side of the enemy')
        self.scenario_A1( dist, napravl)

    def scenario_B1(self):#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
        print('the robot moves to the left 4 steps')
        if self.local.ball_odometry[1] > self.local.coord_odometry[1]:
            if self.local.ball_odometry[1] > 0.4: 
                if self.local.coord_odometry[1] < 0.4:
                    self.motion.near_distance_omni_motion( 1000*(0.4 - self.local.coord_odometry[1]), math.pi/2)
            else:
                self.motion.near_distance_omni_motion( 1000*(self.local.ball_odometry[1] - self.local.coord_odometry[1]), math.pi/2)
        self.turn_Face_To_Guest()

    def scenario_B2(self):#the robot moves to the left and stands on the same axis as the ball and the opponents' goal
        print('the robot moves to the left 4 steps')
        self.scenario_B1()

    def scenario_B3(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        print('the robot moves to the right 4 steps')
        #self.motion.first_Leg_Is_Right_Leg = True
        #self.motion.near_distance_omni_motion( 110, -math.pi/2)
        if self.local.ball_odometry[1] < self.local.coord_odometry[1]:
            if self.local.ball_odometry[1] < -0.4: 
                if self.local.coord_odometry[1] > -0.4:
                    self.motion.near_distance_omni_motion( 1000*(0.4 + self.local.coord_odometry[1]), -math.pi/2)
            else:
                self.motion.near_distance_omni_motion( 1000*(-self.local.ball_odometry[1] + self.local.coord_odometry[1]), -math.pi/2)
        self.turn_Face_To_Guest()

    def scenario_B4(self):#the robot moves to the right and stands on the same axis as the ball and the opponents' goal
        print('the robot moves to the right 4 steps')
        self.scenario_B3()

class Forward:
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0

    def dir_To_Guest(self):
        if self.local.ball_odometry[0] < 0:
            self.direction_To_Guest = 0
        elif self.local.ball_odometry[0] > 0.8 and abs(self.local.ball_odometry[1]) > 0.6:
            self.direction_To_Guest = math.atan(-self.local.ball_odometry[1]/(1.8-self.local.ball_odometry[0]))
        elif self.local.ball_odometry[0] < 1.5 and abs(self.local.ball_odometry[1]) < 0.25:
            if (1.8-self.local.ball_odometry[0]) == 0: self.direction_To_Guest = 0
            else:
                if abs(self.local.ball_odometry[1]) > 0.2:
                    self.direction_To_Guest = math.atan((math.copysign(0.2, self.local.ball_odometry[1])-
                                                       self.local.ball_odometry[1])/(1.8-self.local.ball_odometry[0]))
                else:
                    self.direction_To_Guest = math.atan((0.2* (round(random.random(),0)*2 - 1)-
                                                       self.local.ball_odometry[1])/(1.8-self.local.ball_odometry[0]))
        else:
            self.direction_To_Guest = math.atan(-self.local.coord_odometry[1]/(2.8-self.local.coord_odometry[0]))
        return self.direction_To_Guest

    def turn_Face_To_Guest(self):
        self.dir_To_Guest()
        self.motion.turn_To_Course(self.direction_To_Guest)

class Forward_Vector_Matrix:
    def __init__(self, motion, local, glob):
        self.motion = motion
        self.local = local
        self.glob = glob
        self.direction_To_Guest = 0
        self.kick_Power = 1

    def dir_To_Guest(self):
        if abs(self.local.ball_odometry[0])  >  self.glob.landmarks["FIELD_LENGTH"] / 2:
            ball_x = math.copysign(self.glob.landmarks["FIELD_LENGTH"] / 2, self.local.ball_odometry[0])
        else: ball_x = self.local.ball_odometry[0]
        if abs(self.local.ball_odometry[1])  >  self.glob.landmarks["FIELD_WIDTH"] / 2:
            ball_y = math.copysign(self.glob.landmarks["FIELD_WIDTH"] / 2, self.local.ball_odometry[1])
        else: ball_y = self.local.ball_odometry[1]
        col = math.floor((ball_x + self.glob.landmarks["FIELD_LENGTH"] / 2) / (self.glob.landmarks["FIELD_LENGTH"] / self.glob.COLUMNS))
        row = math.floor((- ball_y + self.glob.landmarks["FIELD_WIDTH"] / 2) / (self.glob.landmarks["FIELD_WIDTH"] / self.glob.ROWS))
        if col >= self.glob.COLUMNS : col = self.glob.COLUMNS - 1
        if row >= self.glob.ROWS : row = self.glob.ROWS -1
        self.direction_To_Guest = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2 + 1] / 40
        self.kick_Power = self.glob.strategy_data[(col * self.glob.ROWS + row) * 2]
        #print('direction_To_Guest = ', math.degrees(self.direction_To_Guest))
        return row, col

    def turn_Face_To_Guest(self):
        self.dir_To_Guest()
        self.motion.turn_To_Course(self.direction_To_Guest)

class Player():
    def __init__(self, role, second_pressed_button, glob, motion, local):
        self.role = role   #'goalkeeper', 'penalty_Goalkeeper', 'forward', 'penalty_Shooter'
        self.second_pressed_button = second_pressed_button
        self.glob = glob
        self.motion = motion
        self.local = local
        self.g = None
        self.f = None
        if self.glob.SIMULATION == 5:
            from Soccer.Motion.class_stm_channel import STM_channel
            from Soccer.Vision.class_Vision_RPI import Vision_RPI
            self.STM_channel = STM_channel
            self.Vision_RPI = Vision_RPI

    def play_game(self):
        if self.role == 'goalkeeper': self.goalkeeper_main_cycle()
        if self.role == 'penalty_Goalkeeper': self.penalty_Goalkeeper_main_cycle()
        if self.role == 'side_to_side': self.side_to_side_main_cycle()
        if self.role == 'forward': self.forward_main_cycle(self.second_pressed_button)
        if self.role == 'forward_v2': self.forward_v2_main_cycle()
        if self.role == 'marathon':  self.marathon_main_cycle()
        if self.role == 'penalty_Shooter': self.penalty_Shooter_main_cycle()
        if self.role == 'run_test': self.run_test_main_cycle(self.second_pressed_button)
        if self.role == 'rotation_test': self.rotation_test_main_cycle()
        if self.role == 'sidestep_test': self.sidestep_test_main_cycle()
        if self.role == 'obstacle_runner': self.obstacle_runner_main_cycle()
        if self.role == 'test_walk': self.test_walk_main_cycle()
        if self.role == 'ball_moving': self.ball_moving_main_cycle()
        if self.role == 'dance': self.dance_main_cycle()
        if self.role == 'basketball': self.basketball_main_cycle(self.second_pressed_button)
        if self.role == 'weight_lifting': self.weight_lifting(self.second_pressed_button)
        if self.role == 'corner_kick_1': self.corner_kick_1_main_cycle()
        if self.role == 'corner_kick_2': self.corner_kick_2_main_cycle()
        if self.role == 'triple_jump': self.triple_jump_main_cycle()
        if self.role == 'sprint': self.sprint(self.second_pressed_button)
        if self.role == 'kick_test': self.kick_test(self.second_pressed_button)
        #print('self.glob.SIMULATION:', self.glob.SIMULATION)
        if [0,1,3].count(self.glob.SIMULATION) == 1:
            self.motion.sim_Stop()
            if self.glob.SIMULATION != 0:
                self.motion.print_Diagnostics()
                pass
            self.motion.sim_Disable()
        #sys.exit(1)

    def rotation_test_main_cycle(self):
        number_Of_Cycles = 10
        stepLength = 0
        sideLength = 0
        self.motion.params['ROTATION_YIELD_RIGHT'] = 0.23
        self.motion.params['ROTATION_YIELD_LEFT'] = 0.23
        self.motion.refresh_Orientation()
        first_yaw_measurement = self.motion.imu_body_yaw()
        rotation = -0.23
        self.motion.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            self.motion.walk_Cycle(stepLength,sideLength, rotation, cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()
        self.motion.refresh_Orientation()
        second_yaw_measurement = self.motion.imu_body_yaw()
        time.sleep(2)
        rotation = 0.23
        self.motion.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            self.motion.walk_Cycle(stepLength,sideLength, rotation, cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()
        self.motion.refresh_Orientation()
        third_yaw_measurement = self.motion.imu_body_yaw()
        if self.motion.glob.SIMULATION == 5:
            filename = "/home/pi/Desktop/" + "Init_params/Real/Real_params.json"
        else:
            filename = self.glob.current_work_directory + "Init_params/Sim/" + "Sim_params.json"
        with open(filename, "r") as f:
            params = json.loads(f.read())
            rotation_yield_right = abs(second_yaw_measurement - first_yaw_measurement) / 10
            rotation_yield_left = abs(third_yaw_measurement - second_yaw_measurement) / 10
            #rotation_yield = (rotation_yield_right + rotation_yield_left)/2
            params['ROTATION_YIELD_RIGHT'] = round(rotation_yield_right, 3)
            params['ROTATION_YIELD_LEFT'] = round(rotation_yield_left, 3)
        jsonstring = '{\n"BODY_TILT_AT_WALK": ' + str(params["BODY_TILT_AT_WALK"]) \
                    + ',\n"SOLE_LANDING_SKEW": ' + str(params["SOLE_LANDING_SKEW"]) \
                    + ',\n"BODY_TILT_AT_KICK": ' + str(params["BODY_TILT_AT_KICK"]) \
                    + ',\n"ROTATION_YIELD_RIGHT": ' + str(params["ROTATION_YIELD_RIGHT"]) \
                    + ',\n"ROTATION_YIELD_LEFT": ' + str(params["ROTATION_YIELD_LEFT"]) \
                    + ',\n"MOTION_SHIFT_TEST_X": ' + str(params["MOTION_SHIFT_TEST_X"]) \
                    + ',\n"MOTION_SHIFT_TEST_Y": ' + str(params["MOTION_SHIFT_TEST_Y"]) \
                    + ',\n"SIDE_STEP_RIGHT_TEST_RESULT": ' + str(params["SIDE_STEP_RIGHT_TEST_RESULT"]) \
                    + ',\n"SIDE_STEP_LEFT_TEST_RESULT": ' + str(params["SIDE_STEP_LEFT_TEST_RESULT"]) \
                    + ',\n"RUN_TEST_10_STEPS": ' + str(params["RUN_TEST_10_STEPS"]) \
                    + ',\n"RUN_TEST_20_STEPS": ' + str(params["RUN_TEST_20_STEPS"]) \
                    + ',\n"KICK_ADJUSTMENT_DISTANCE_1": ' + str(params["KICK_ADJUSTMENT_DISTANCE_1"]) \
                    + ',\n"KICK_ADJUSTMENT_DISTANCE_2": ' + str(params["KICK_ADJUSTMENT_DISTANCE_2"]) \
                    + ',\n"KICK_OFFSET_OF_BALL": ' + str(params["KICK_OFFSET_OF_BALL"]) \
                    + ',\n"IMU_DRIFT_IN_DEGREES_DURING_6_MIN_MEASUREMENT": ' + str(params["IMU_DRIFT_IN_DEGREES_DURING_6_MIN_MEASUREMENT"]) \
                    + '\n}'
        with open(filename, "w") as f:
            f.write(jsonstring)
            #json.dump(params_new, f)
        self.motion.turn_To_Course(math.pi/3*2)
        self.motion.turn_To_Course(0)

    def run_test_main_cycle(self, pressed_button, stepLength = 64):
        if pressed_button == 'head_tilt_calibration':
            #execfile("Head_Tilt_Calibration.py")
            if self.motion.head_Tilt_Calibration():
                self.motion.rcb.motionPlay(25)          # zummer
            else: 
                self.motion.rcb.motionPlay(25)          # zummer
                time.sleep(0.2)
                self.motion.rcb.motionPlay(25)          # zummer
            return
        self.motion.head_Return(0, self.motion.neck_play_pose)
        if pressed_button == 'side_step_right' or pressed_button =='side_step_left' :
            self.sidestep_test_main_cycle(pressed_button)
            return
        if pressed_button == 'rotation_right' or pressed_button =='rotation_left' :
            self.rotation_test_main_cycle()
            return

        stepLength = 64
        self.motion.gaitHeight = 180
        if pressed_button == 'spot_run': stepLength = 0
        number_Of_Cycles = 20
        self.motion.amplitude = 32
        if pressed_button == 'short_run': number_Of_Cycles = 10
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            rotation = 0 + invert * self.motion.imu_body_yaw() * 1.1
            #if rotation > 0: rotation *= 1.5
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()

    def sidestep_test_main_cycle(self, pressed_button):
        number_Of_Cycles = 20
        stepLength = 0 #64
        sideLength = 20
        if pressed_button == 'side_step_left':
            self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        #if pressed_button == 'side_step_left': sideLength = -20
        #else: sideLength = 20
        #invert = 1
        self.motion.walk_Initial_Pose()
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            #if cycle ==0 : stepLength1 = stepLength/3
            #if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            rotation = 0 - invert * self.motion.imu_body_yaw() * 1.1
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def forward_main_cycle(self, pressed_button):
        #self.glob.with_pf = False
        self.f = Forward_Vector_Matrix(self.motion, self.local, self.glob)
        if self.glob.SIMULATION == 5:
            self.motion.rcb.motionPlay(25)          # zummer
            labels = [[], [], [], ['start', 'start_later'], []]
            pressed_button = self.motion.push_Button(labels)
        second_player_timer = time.time()
        #self.glob.vision.camera_thread.start()
        #self.motion.control_Head_motion_thread.start()
        if pressed_button == 'start':
            self.motion.kick_off_ride()
            first_look_point = None
        else:
            first_look_point= self.local.ball_odometry
        while (True):
            if (time.perf_counter() - self.motion.start_point_for_imu_drift) > 360:
                self.motion.turn_To_Course(0)
                self.motion.turn_To_Course(0, accurate = True)
                if self.glob.SIMULATION == 5:
                    for i in range(5):
                        self.motion.rcb.motionPlay(25)
                        self.motion.pause_in_ms(400)
                break
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            if self.glob.SIMULATION == 5:
                if self.glob.camera_down_Flag == True:
                    print('Camera resetting')
                    self.glob.camera_down_Flag = False
                    self.glob.vision.camera.picam2.close()
                    self.glob.vision.event.set()
                    new_stm_channel  = self.STM_channel(self.glob)
                    self.glob.stm_channel = new_stm_channel
                    self.glob.rcb = self.glob.stm_channel.rcb
                    new_vision = self.Vision_RPI(self.glob)
                    self.glob.vision = new_vision
                    self.motion.vision = self.glob.vision
                    self.local.vision = self.glob.vision
                    self.glob.vision.camera_thread.start()
            #success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True, with_Localization = False,
            #                                                                  very_Fast = True, first_look_point=first_look_point)
            #time.sleep(1) # this is to look around for ball 
            self.glob.vision.detect_Ball_in_One_Shot()
            if self.glob.robot_see_ball > 0: 
                self.glob.ball_coord = self.local.ball_odometry
            self.glob.pf_coord = self.local.coord_odometry
            time_elapsed = time.time() - second_player_timer
            if self.glob.SIMULATION == 5: frozen_time = 10
            else: frozen_time = 10
            if pressed_button == 'start_later' and time_elapsed < frozen_time : 
                #if self.glob.SIMULATION == 1: self.motion.sim_simxSynchronousTrigger(self.motion.clientID)
                time.sleep(0.02)
                continue
            self.f.dir_To_Guest()
            print('direction_To_Guest = ', round(math.degrees(self.f.direction_To_Guest)), 'degrees')
            print('coord =', round(self.local.coord_odometry[0],2), round(self.local.coord_odometry[1],2), 'ball =', round(self.local.ball_odometry[0],2), round(self.local.ball_odometry[1],2))
            if self.glob.robot_see_ball <= 0:
                print('Seek ball')
                self.motion.turn_To_Course(self.local.coord_odometry[2]+ 2 * math.pi / 3)
                continue
            player_from_ball_yaw = coord2yaw(self.local.coord_odometry[0] - self.local.ball_odometry[0],
                                                          self.local.coord_odometry[1] - self.local.ball_odometry[1]) - self.f.direction_To_Guest
            player_from_ball_yaw = self.norm_yaw(player_from_ball_yaw)
            player_in_front_of_ball = -math.pi/2 < player_from_ball_yaw < math.pi/2
            player_in_fast_kick_position = (player_from_ball_yaw > 2.5 or player_from_ball_yaw < -2.5) and self.glob.ball_distance < 0.6
            if self.glob.ball_distance > 0.35  and not player_in_fast_kick_position:
                if self.glob.ball_distance > 3: stop_Over = True
                else: stop_Over = False
                direction_To_Ball = math.atan2((self.local.ball_odometry[1] - self.local.coord_odometry[1]), (self.local.ball_odometry[0] - self.local.coord_odometry[0]))
                print('napravl :', self.glob.ball_course)
                print('direction_To_Ball', direction_To_Ball)
                #self.motion.far_distance_plan_approach(self.local.ball_odometry, self.f.direction_To_Guest, stop_Over = stop_Over)
                self.motion.far_distance_straight_approach(self.local.ball_odometry, direction_To_Ball, stop_Over = stop_Over)
                #self.go_Around_Ball(dist, napravl)
                continue
            if player_in_front_of_ball or not player_in_fast_kick_position:
                self.go_Around_Ball(self.glob.ball_distance, self.glob.ball_course)
                continue
            if player_in_fast_kick_position:
                self.motion.turn_To_Course(self.f.direction_To_Guest)
                small_kick = False
                if self.f.kick_Power > 1: small_kick = True
                success_Code = self.motion.near_distance_ball_approach_and_kick_streaming(self.f.direction_To_Guest, small_kick = small_kick)

    def goalkeeper_main_cycle(self):
        def ball_position_is_dangerous(row, col):
            danger = False
            danger = (col <= (round(self.glob.COLUMNS / 3) - 1))
            if ((row <= (round(self.glob.ROWS / 3) - 1) or row >= round(self.glob.ROWS * 2 / 3)) and col == 0) or (col == 1 and (row == 0 or row == (self.glob.ROWS -1))):
               danger = False
            return danger
        second_player_timer = time.time()
        self.f = Forward_Vector_Matrix(self.motion, self.local, self.glob)
        #self.motion.near_distance_omni_motion(400, 0)                    # get out from goal
        fast_Reaction_On = True
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                #self.local.coordinate_fall_reset()
            #if self.local.ball_odometry[0] <= 0.15:
            #    success_Code, napravl, dist, speed =  self.motion.watch_Ball_In_Pose()
            #else:
            #    success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = fast_Reaction_On)
            time.sleep(1) # this is to look around for ball
            napravl, dist, speed = self.glob.ball_course, self.glob.ball_distance, self.glob.ball_speed
            if abs(speed[0]) > 0.02 and dist < 1 :                         # if dangerous tangential speed
                fast_Reaction_On = True
                if speed[0] > 0:
                    if self.local.coord_odometry[1] < 0.35:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceL')
                else:
                    if self.local.coord_odometry[1] > -0.35:
                        self.motion.play_Soft_Motion_Slot(name ='PenaltyDefenceR')
                self.motion.pause_in_ms(3000)
                self.motion.falling_Test()
                continue
            if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                fast_Reaction_On = True
                self.motion.play_Soft_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                self.motion.play_Soft_Motion_Slot(name = 'PenaltyDefenceF')
                self.motion.pause_in_ms(3000)
                self.motion.play_Soft_Motion_Slot(name = 'Get_Up_From_Defence')
                continue
            if (time.time() - second_player_timer) < 10 : continue
            row, col = self.f.dir_To_Guest()
            #print('direction_To_Guest = ', math.degrees(self.f.direction_To_Guest), 'degrees')
            #print('goalkeeper coord =', self.glob.pf_coord, 'ball =', self.glob.ball_coord, 'row =', row, 'col =', col, 'ball_position_is_dangerous =', ball_position_is_dangerous(row,col))
            if dist == 0 and self.glob.robot_see_ball <= 0:
                if self.local.coord_odometry[0] > -1.3:
                    print('goalkeeper turn_To_Course(pi*2/3)')
                    self.motion.turn_To_Course(self.local.coord_odometry[2]+ 2 * math.pi / 3)
                continue
            if ball_position_is_dangerous(row, col):
                fast_Reaction_On = True
                player_from_ball_yaw = coord2yaw(self.local.coord_odometry[0] - self.local.ball_odometry[0], self.local.coord_odometry[1] - self.local.ball_odometry[1]) - self.f.direction_To_Guest
                player_from_ball_yaw = self.norm_yaw(player_from_ball_yaw)
                player_in_front_of_ball = -math.pi/2 < player_from_ball_yaw < math.pi/2
                player_in_fast_kick_position = (player_from_ball_yaw > 2 or player_from_ball_yaw < -2) and dist < 0.6
                if dist > 0.35 and not player_in_fast_kick_position:
                    if dist > 3: stop_Over = True
                    else: stop_Over = False
                    print('goalkeeper far_distance_plan_approach')
                    direction_To_Ball = math.atan2((self.local.ball_odometry[1] - self.local.coord_odometry[1]), (self.local.ball_odometry[0] - self.local.coord_odometry[0]))
                    self.motion.far_distance_straight_approach(self.local.ball_odometry, direction_To_Ball, stop_Over = stop_Over)
                    #self.f.turn_Face_To_Guest()
                    continue
                if player_in_front_of_ball or not player_in_fast_kick_position:
                    self.go_Around_Ball(dist, napravl)
                    continue
                print('goalkeeper turn_To_Course(direction_To_Guest)')
                self.motion.turn_To_Course(self.f.direction_To_Guest)
                small_kick = False
                if self.f.kick_Power > 1: small_kick = True
                print('goalkeeper near_distance_ball_approach_and_kick')
                success_Code = self.motion.near_distance_ball_approach_and_kick(self.f.direction_To_Guest, strong_kick = False, small_kick = small_kick)

            else:
                fast_Reaction_On = False
                duty_x_position =  min((-self.glob.landmarks['FIELD_LENGTH']/2 + 0.4),(self.local.ball_odometry[0]-self.glob.landmarks['FIELD_LENGTH']/2)/2)
                duty_y_position = self.local.ball_odometry[1] * (duty_x_position + self.glob.landmarks['FIELD_LENGTH']/2) / (self.local.ball_odometry[0] + self.glob.landmarks['FIELD_LENGTH']/2)
                duty_distance = math.sqrt((duty_x_position - self.local.coord_odometry[0])**2 + (duty_y_position - self.local.coord_odometry[1])**2)
                #print('duty_x_position =', duty_x_position, 'duty_y_position =', duty_y_position)
                if duty_distance < 0.2 : continue
                elif duty_distance <  3: #   0.6 :
                    print('goalkeeper turn_To_Course(0)')
                    self.motion.turn_To_Course(0)
                    duty_direction = coord2yaw(duty_x_position - self.local.coord_odometry[0], duty_y_position - self.local.coord_odometry[1])
                    print('goalkeeper near_distance_omni_motion')
                    self.motion.near_distance_omni_motion(duty_distance * 1000, duty_direction)
                    print('goalkeeper turn_To_Course(0)')
                    self.motion.turn_To_Course(0)
                else:
                    direction_To_Duty = math.atan2((duty_y_position - self.local.coord_odometry[1]), (duty_x_position - self.local.coord_odometry[0]))
                    self.motion.far_distance_straight_approach([duty_x_position , duty_y_position], direction_To_Duty, gap = 0, stop_Over = False)
                    self.motion.turn_To_Course(0)


    def penalty_Shooter_main_cycle(self):
        self.f = Forward(self.motion, self.local, self.glob)
        first_shoot = True
        first_look_point = [0.9, 0]
        self.glob.vision.camera_thread.start()
        self.motion.control_Head_motion_thread.start()
        self.motion.kick_off_ride()
        while (True):
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
            #success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True, with_Localization = False,
            #                                                                  very_Fast = False, first_look_point=first_look_point )
            if self.glob.robot_see_ball > 0: self.local.ball_position_calculation()
            first_look_point = self.local.ball_odometry
            self.f.dir_To_Guest()
            print('ball_coord = ', self.local.ball_odometry)
            print('direction_To_Guest = ', math.degrees(self.f.direction_To_Guest), 'degrees')
            if self.glob.robot_see_ball <= 0:
                self.motion.turn_To_Course(self.local.coord_odometry[2]+ 2 * math.pi / 3)
                continue
            player_from_ball_yaw = coord2yaw(self.local.coord_odometry[0] - self.local.ball_odometry[0],
                                                          self.local.coord_odometry[1] - self.local.ball_odometry[1]) - self.f.direction_To_Guest
            player_from_ball_yaw = self.norm_yaw(player_from_ball_yaw)
            player_in_front_of_ball = -math.pi/2 < player_from_ball_yaw < math.pi/2
            player_in_fast_kick_position = (player_from_ball_yaw > 2.5 or player_from_ball_yaw < -2.5) and self.glob.ball_distance < 0.6
            if self.glob.ball_distance > 0.35  and not player_in_fast_kick_position:
                if self.glob.ball_distance> 3: stop_Over = True
                else: stop_Over = False
                direction_To_Ball = math.atan2((self.local.ball_odometry[1] - self.local.coord_odometry[1]), (self.local.ball_odometry[0] - self.local.coord_odometry[0]))
                self.motion.far_distance_straight_approach(self.local.ball_odometry, direction_To_Ball, stop_Over = stop_Over)
                continue
            if player_in_front_of_ball or not player_in_fast_kick_position:
                self.go_Around_Ball(self.glob.ball_distance, self.glob.ball_course)
                continue
            self.motion.turn_To_Course(self.f.direction_To_Guest)
            #if first_shoot:
            success_Code = self.motion.near_distance_ball_approach_and_kick_streaming(self.f.direction_To_Guest)
            first_shoot = False


    def penalty_Goalkeeper_main_cycle(self):
        self.g = GoalKeeper(self.motion, self.local, self.glob)
        self.glob.obstacleAvoidanceIsOn = False
        first_Get_Up = True
        while (True):
            dist = -1.0
            if self.motion.falling_Flag != 0:
                if self.motion.falling_Flag == 3: break
                self.motion.falling_Flag = 0
                self.local.coordinate_fall_reset()
                self.g.turn_Face_To_Guest()
                if first_Get_Up:
                    first_Get_Up = False
                    self.g.goto_Center()
            while(dist < 0):
                a, napravl, dist, speed = self.motion.watch_Ball_In_Pose(penalty_Goalkeeper = True)
                napravl = self.glob.ball_course
                dist = self.glob.ball_distance
                speed = self.glob.ball_speed
                #print('speed = ', speed, 'dist  =', dist , 'napravl =', napravl)
                if abs(speed[0]) > 0.002 and dist < 1 :                         # if dangerous tangential speed
                    if speed[0] > 0:
                        self.motion.play_Motion_Slot(name ='PenaltyDefenceL')
                    else:
                        self.motion.play_Motion_Slot(name ='PenaltyDefenceR')
                    continue
                if speed[1] < - 0.01 and dist < 1.5 :                          # if dangerous front speed
                    self.motion.play_Motion_Slot(name = 'PanaltyDefenceReady_Fast')
                    self.motion.play_Motion_Slot(name = 'PenaltyDefenceF')
                    self.motion.pause_in_ms(5000)
                    self.motion.play_Motion_Slot(name = 'Get_Up_From_Defence')

                if (dist == 0 and napravl == 0) or dist > 2.5:
                    continue
                old_neck_pan, old_neck_tilt = self.motion.head_Up()
                if (dist <= 0.5         and 0 <= napravl <= math.pi/4):         self.g.scenario_A1( dist, napravl)
                if (dist <= 0.5         and math.pi/4 < napravl <= math.pi/2):  self.g.scenario_A2( dist, napravl)
                if (dist <= 0.5         and 0 >= napravl >= -math.pi/4):        self.g.scenario_A3( dist, napravl)
                if (dist <= 0.7         and -math.pi/4 > napravl >= -math.pi/2): self.g.scenario_A4( dist, napravl)
                if ((0.5 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/18 <= napravl <= math.pi/4)): self.g.scenario_B1()
                if ((0.5 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (math.pi/4 < napravl <= math.pi/2)): self.g.scenario_B2()
                if ((0.5 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/18 >= napravl >= -math.pi/4)): self.g.scenario_B3()
                if ((0.7 < dist < self.glob.landmarks['FIELD_LENGTH']/2) and (-math.pi/4 > napravl >= -math.pi/2)): self.g.scenario_B4()
                self.motion.head_Return(old_neck_pan, old_neck_tilt)

    def go_Around_Ball(self, dist, napravl):
        print('go_Around_Ball')
        turning_radius = 0.20 # meters
        #first_look_point= self.local.ball_odometry
        #success_Code, napravl, dist, speed = self.motion.seek_Ball_In_Pose(fast_Reaction_On = True, with_Localization = False,
        #                                                                  very_Fast = True, first_look_point=first_look_point)
        #if success_Code != True: return
        if dist > 0.5: return
        correction_x = dist * math.cos(napravl)
        correction_y = dist * math.sin(napravl)
        alpha = self.f.direction_To_Guest - self.local.coord_odometry[2]
        alpha = self.motion.norm_yaw(alpha)
        initial_body_yaw = self.local.coord_odometry[2]
        correction_napravl = math.atan2(correction_y, (correction_x - turning_radius))
        correction_dist = math.sqrt(correction_y**2 + (correction_x - turning_radius)**2)
        old_neck_pan, old_neck_tilt = self.motion.head_Up()
        if napravl * alpha > 0:
            if napravl > 0: self.motion.first_Leg_Is_Right_Leg = False
            self.motion.walk_Initial_Pose()
            if self.glob.ball_distance > 0.6: self.motion.falling_Flag = 3
            self.motion.turn_To_Course(self.local.coord_odometry[2] + napravl, one_Off_Motion = False)
            if self.glob.ball_distance > 0.6: self.motion.falling_Flag = 3
            self.motion.walk_Restart()
            if self.glob.ball_distance > 0.6: self.motion.falling_Flag = 3
            self.motion.near_distance_omni_motion((dist - turning_radius) * 1000 , 0, one_Off_Motion = False)
            alpha = self.motion.norm_yaw(alpha - napravl)
            initial_body_yaw += napravl
        else:
            if correction_napravl > 0: self.motion.first_Leg_Is_Right_Leg = False
            self.motion.walk_Initial_Pose()
            if self.glob.ball_distance > 0.6: self.motion.falling_Flag = 3
            self.motion.near_distance_omni_motion(correction_dist*1000, correction_napravl, one_Off_Motion = False)
        if alpha >= 0:
            if self.motion.first_Leg_Is_Right_Leg == False:
                change_legs = True
            else:
                change_legs = False
                self.motion.walk_Restart()
            self.motion.first_Leg_Is_Right_Leg = True
            side_step_yield = self.motion.side_step_right_yield
            invert = 1
        else:
            if self.motion.first_Leg_Is_Right_Leg == True:
                change_legs = True
            else:
                change_legs = False
                self.motion.walk_Restart()
            self.motion.first_Leg_Is_Right_Leg = False
            side_step_yield = self.motion.side_step_left_yield
            invert = -1
        #print('6self.motion.first_Leg_Is_Right_Leg:', self.motion.first_Leg_Is_Right_Leg)
        yaw_increment_at_side_step =  math.copysign(2 * math.asin(side_step_yield / 2 / (turning_radius * 1000)), alpha)
        number_Of_Cycles = int(round(abs(alpha / yaw_increment_at_side_step)))+1
        stepLength = 0
        sideLength = 20
        for cycle in range(number_Of_Cycles):
            self.motion.refresh_Orientation()
            rotation = initial_body_yaw + cycle * yaw_increment_at_side_step - self.motion.imu_body_yaw() * 1.1
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            if self.glob.ball_distance > 0.6: self.motion.falling_Flag = 3
            self.motion.walk_Cycle(stepLength, sideLength, invert*rotation, cycle, number_Of_Cycles)
        if self.motion.falling_Flag == 3 : self.motion.falling_Flag = 0 
        self.motion.walk_Final_Pose()
        self.motion.first_Leg_Is_Right_Leg = True

    def basketball_main_cycle(self, pressed_button):
        
        intercom = self.glob.stm_channel.zubr       # used for communication between head and zubr-controller with memIGet/memISet commands

        #throw = [
        #        [ 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 450, 0, 4700, 2667, 0, 0, 0, 0, 0, 0 ],
        #        [ 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 450, 0, 4700, 2667, 0, 0, 0, 0, 0, 0 ],
        #        [ 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2000, 0, 4700, 0, 0, 0, 0, 0, 0, 0 ],
        #        [ 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2000, 0, 4700, 0, 0, 0, 0, 0, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
        #        ]
        #throw[2][18] -= int(self.motion.params['BASKETBALL_DISTANCE'])
        #throw[3][18] -= int(self.motion.params['BASKETBALL_DISTANCE'])
        #pickUp = [
        #        [ 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
        #        [ 100, 0, -900, 0, -3300, 0, 0, 0, 2667, 0, -3500, 0, 0, 900, 0, 3300, 0, 0, 0, -2667, 0, 3500, 0, 0, 0, 0, 0, 0 ],
        #        [ 50, 0, -900, 0, -3300, 0, 0, 600, 2300, 385, -3500, 0, 0, 900, 0, 3300, 0, 0, -600, -2300, -385, 3500, 0, 0, 0, 0, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 600, 2667, 385, -2667, 0, 0, 0, 0, 0, 0, 0, -600, -2667, -385, 2667, 0, 0, 0, 0, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 600, 3000, 385, -3000, 0, 0, 0, 0, 0, 0, 0, -700, -2000, -385, 2500, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 700, 3425, 385, -3300, 0, 0, 0, 0, 0, 0, 0, -900, -1500, -385, 2200, 0, 0, 0, 0, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 910, 2667, 600, -2600, 0, 0, 0, 0, 0, 0, 0, -2100, -600, -680, 1000, 0, 0, 0, 0, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 0, 2667, 600, -2600, 0, 0, 0, 0, 0, 0, 0, -2100, 0, -680, 1000, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1100, 0, -680, 2000, 0, 0, 0, 0, 0, 0 ],
        #        [ 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -433, 0, -680, 2667, 0, 0, 0, 0, 0, 0 ],
        #        [ 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -433, 0, 2500, 2667, 0, 0, 0, 0, 0, 0 ],
        #        [ 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -350, 0, 4700, 2667, 0, 0, 0, 0, 0, 0 ]
        #        ]
        #for i in range(2, 8,1):
        #    pickUp[i][9] += int(self.motion.params['BASKETBALL_CLAMPING'])
        #    pickUp[i][20] -= int(self.motion.params['BASKETBALL_CLAMPING'])
        #for i in range(4):
        #    throw[i][19] += int(self.motion.params['BASKETBALL_DIRECTION'])
        if pressed_button == 'start' or pressed_button == 'pick_up_test' :
            #self.motion.play_Soft_Motion_Slot( name = 'pickUp', motion_list = pickUp)
            var = roki2met.roki2met.Basketball_PickUp_v2
            self.glob.rcb.motionPlay(10)                                # Basketball_PickUp
            while True:
                ok, frameCount = intercom.memIGet(var.frameCount)
                if ok: print('frameCount :', frameCount)
                else: print(intercom.GetError())
                if frameCount == 1: break
                time.sleep(0.25)
            intercom.memISet(var.clamping, int(self.motion.params['BASKETBALL_CLAMPING']))         # clamping gap for ball gripping -50 best value
            intercom.memISet(var.steps, int(self.motion.params['BASKETBALL_SIDE_SHIFT_STEPS']))    # side shift steps to provide 80mm shifting to right. 17 is the best value
            intercom.memISet(var.pitStop, 1)                                                       # ignition
            time.sleep(35)

        if pressed_button == 'start' or pressed_button == 'throw_test' or pressed_button == 'throw_control':
            var = roki2met.roki2met.Basketball_Throw
            
            int_voltage = self.motion.stm_channel.read_voltage_from_body()[1]
            voltage = int_voltage / 270.2
            print("voltage = ", round(voltage, 2), " 'BASKETBALL_DISTANCE': ", 
                  int(self.motion.params['BASKETBALL_DISTANCE']))
            #self.motion.play_Soft_Motion_Slot( name = 'throw', motion_list = throw)
            self.glob.rcb.motionPlay(9)                                # Basketball_Throw
            while True:
                ok, frameCount = intercom.memIGet(var.frameCount)
                if ok: print('frameCount :', frameCount)
                else: print(intercom.GetError())
                if frameCount == 1: break
                time.sleep(0.25)
            
            intercom.memISet(var.pitStop, 1)                                                       # ignition
            time.sleep(3)
            for i in range(100):
                result, displacement = self.glob.vision.detect_Basket_in_One_Shot()
                if result: break
            if result:
                os.system("espeak -ven-m1 -a"+ '200' + " " + "'I see basket'")
            else: 
                os.system("espeak -ven-m1 -a"+ '200' + " " + "'I don't see basket'")
            time.sleep(3)
            corrected_direction = int(self.motion.params['BASKETBALL_DIRECTION']) + int(displacement/10 /360 * 16384)
            if pressed_button == 'start' or pressed_button == 'throw_control':
                if int(self.motion.params['BASKETBALL_BATTERY_NUMBER']) == 1:
                    voltage_correction = 10469.14829 + -1506.46893 * voltage + 55.11101 * voltage ** 2  
                elif int(self.motion.params['BASKETBALL_BATTERY_NUMBER']) == 3:
                    voltage_correction = 50435.37775 + -7951.56631 * voltage + 314.69345 * voltage ** 2
                else: voltage_correction = 0
                corrected_distance = int(self.motion.params['BASKETBALL_DISTANCE_CORRECTION'] + int(voltage_correction))
            else:
                corrected_distance = int(self.motion.params['BASKETBALL_DISTANCE'])
            intercom.memISet(var.distance, corrected_distance)         # start acceleration angle -350 best value
            intercom.memISet(var.direction, corrected_direction)       # direction to correct 200 best value
            intercom.memISet(var.startStop, 1)                                                       # ignition
            time.sleep(3)
            if pressed_button == 'throw_test':
                labels = [[], [], [], ['good', 'Bad'], []]
                pressed_button = self.motion.push_Button(labels, message = "'Give me feed back'")
                if pressed_button == 'good':
                    today = datetime.date.today()
                    try:
                        records = np.load("basketball_records.npy")
                    except Exception:
                        records = np.zeros((1,6), dtype = np.int16)
                    record = np.array([[int(today.year), int(today.month), int(today.day),
                                        self.motion.params['BASKETBALL_BATTERY_NUMBER'], int_voltage,
                                        int(self.motion.params['BASKETBALL_DISTANCE'])]], dtype = np.int16)
                    new_records = np.append(records, record, axis=0)
                    np.save("basketball_records.npy", new_records)


    def dance_main_cycle(self):
        if self.glob.SIMULATION == 5:
            # while True:
            #     self.motion.refresh_Orientation()
            #     print('\rbody pitch: ', round(self.motion.body_euler_angle['pitch'],3), '\tbody roll : ', round(self.motion.body_euler_angle['roll'], 3),
            #           '\tbody yaw: ', round(self.motion.body_euler_angle['yaw'], 3),
            #           '\thead pitch: ', round(self.motion.euler_angle['pitch'], 3),
            #           '\thead roll : ', round(self.motion.euler_angle['roll'], 3),
            #           '\thead yaw: ', round(self.motion.euler_angle['yaw'], 3), end='' )
            #     time.sleep(1)
                
            self.motion.play_Soft_Motion_Slot( name = 'Basketball3')
#             while True:
#                 successCode, u10 = self.motion.rcb.getUserParameter(10)
#                 #time.sleep(2)
#                 if successCode and u10 == 1:
#                     self.motion.rcb.motionPlay(26)
#                     for i in range(10):
#                         self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
#                     self.motion.play_Soft_Motion_Slot( name = 'Dance_7-1')
#                     self.motion.play_Soft_Motion_Slot( name = 'Dance_7-2')
#                     for i in range(9):
#                         self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
#                     self.motion.play_Soft_Motion_Slot( name = 'Dance_2')
#                     for i in range(2):
#                         self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
#                     self.motion.play_Soft_Motion_Slot( name = 'Dance_4')
#                     self.motion.rcb.setUserParameter(10,0)
        else:
            for i in range(10):
                self.motion.play_Soft_Motion_Slot( name = 'Dance_6_1')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_7')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_2')
            self.motion.play_Soft_Motion_Slot( name = 'Dance_4')

    def sprint(self, second_pressed_button):
        self.motion.params['SPRINT_HIP_TILT'] = 500
        self.motion.params['SPRINT_STEP_LENGTH'] = 40
        self.motion.params['SPRINT_GAIT_HEIGHT'] = 135
        self.motion.params['SPRINT_STEP_HEIGHT'] = 30
        self.motion.params['SPRINT_FPS'] = 2
        self.motion.params['SPRINT_UGOL_TORSA'] = 0.1
        if self.glob.SIMULATION == 5:
            from Soccer.Vision import lookARUCO

            # Pipeline variables
            size = Value('i', 0)       # horizontal size of ARUCO code on picture
            side_shift = Value('i', 0) # horizontal shift of ARUCO code from center of picture 
            stopFlag = Value(c_bool, False)
            #cx = Value('i', 0)
            #cy = Value('i', 0)
            aruco_angle_horizontal = Value('d', 0)

            # Process for Vision Pipeline
            cam_proc = Process(target=lookARUCO.camera_process, args=(size, side_shift,aruco_angle_horizontal, stopFlag), daemon = True)
            # start Process of Vision Pipeline
            cam_proc.start()
            #cam_proc.join()
            pid = cam_proc.pid
            with open('/dev/shm/process.txt', 'w') as process_file:
                print(str(pid), file= process_file)
            process_file.close()

            var = roki2met.roki2met.sprint_v4
            intercom = self.glob.stm_channel.zubr       # used for communication between head and zubr-controller with memIGet/memISet commands
            self.glob.rcb.motionPlay(23)

            while True:
                # wait until motion paramenetrs initiated 
                while True:
                    ok, restart_flag = intercom.memIGet(var.restart_flag)
                    if ok: print('restart_flag :', restart_flag)
                    else: print(intercom.GetError())
                    if restart_flag == 0: break
                    time.sleep(0.25)
            
                # write motion parameters to zubr-controller motion
                intercom.memISet(var.orderFromHead, 0)              #  0 - no order, 1 - straight forward, 2 - to left, 3- to right, 4 - reverse back
                intercom.memISet(var.cycle_number, 33)
                intercom.memISet(var.hipTilt, self.motion.params['SPRINT_HIP_TILT'])
                intercom.memISet(var.fps, self.motion.params['SPRINT_FPS'])
                intercom.memISet(var.stepLengthOrder, self.motion.params['SPRINT_STEP_LENGTH'])
                intercom.memISet(var.gaitHeight, self.motion.params['SPRINT_GAIT_HEIGHT'])
                intercom.memISet(var.stepHeight, self.motion.params['SPRINT_STEP_HEIGHT'])
                intercom.memFSet(var.ugol_torsa, self.motion.params['SPRINT_UGOL_TORSA'])
                intercom.memISet(var.pitStop, 1)                    # 1 - go on, 0 - stop waiting
                #labels = [[], [], [], ['start'], []]
                #pressed_button = self.motion.push_Button(labels)
                #intercom.memISet(var.startStop, 1)
                while True:
                    ok, restart_flag = intercom.memIGet(var.restart_flag)
                    if ok: print('restart_flag :', restart_flag)
                    else: print(intercom.GetError())
                    if restart_flag == 1: break
                    time.sleep(0.25)
                while not stopFlag.value :
                    ok, restart_flag = intercom.memIGet(var.restart_flag)
                    if restart_flag == 0: break
                    aruco_size = size.value
                    aruco_shift = side_shift.value
                    #aruco_cx = cx.value
                    #aruco_cy = cy.value
                    #u, v = self.glob.vision.undistort_points(aruco_cx, aruco_cy)
                    #aruco_angle_horizontal = math.atan((self.glob.vision.undistort_cx -u)/ self.glob.vision.focal_length_horizontal)
                    rotationFromHead = aruco_angle_horizontal.value
                    print('rotationFromHead: ', rotationFromHead)
                    intercom.memFSet(var.rotationFromHead, rotationFromHead)
                    if aruco_size > 90:
                        print('Reverse')
                        intercom.memISet(var.orderFromHead, 4)   
                    elif aruco_size == 0:
                        intercom.memISet(var.orderFromHead, 0) 
                        print('No marker')
                    else:
                        if aruco_shift > 0:
                            print('Go Left')
                            intercom.memISet(var.orderFromHead, 2)
                        elif aruco_shift < 0:
                            print('Go Right')
                            intercom.memISet(var.orderFromHead, 3)
                        else:
                            print('Go Straight')
                            intercom.memISet(var.orderFromHead, 1)
                    time.sleep(0.05)

            return
        self.motion.first_Leg_Is_Right_Leg == True
        timeStep = 1
        number_Of_Cycles = 40
        if timeStep == 1:                   # 10ms
            stepLength = 30 #100
            self.motion.ugol_torsa = 0.3 #0.65
            self.motion.gaitHeight = 135 # 150
            self.motion.stepHeight = 35  # 40
            self.motion.fr1 = 4  #3
            self.motion.fr2 = 9  #6
            self.motion.params['BODY_TILT_AT_WALK'] += 0.020
            self.motion.amplitude = 40
        if timeStep == 2:                      # 20ms
            stepLength = 80 # 200
            self.motion.ugol_torsa = 0.6            # 0.7
            self.motion.gaitHeight = 180
            self.motion.stepHeight = 40
            self.motion.fr1 = 4
            self.motion.fr2 = 6
            self.motion.params['BODY_TILT_AT_WALK'] += 0.020
            self.motion.amplitude = 32
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = 1
        else: invert = -1
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            #self.motion.refresh_Orientation()
            #rotation = 0 - self.motion.imu_body_yaw() * 1.1
            #rotation = self.motion.normalize_rotation(rotation)
            self.motion.walk_Cycle_With_Tors_v3(stepLength1,sideLength, 0 ,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()

    def kick_test(self, second_pressed_button):
        if second_pressed_button == 'regular':
            self.motion.kick(True)
        else:
            self.motion.play_Soft_Motion_Slot(name ='Kick_Right_v3')
        if self.glob.SIMULATION == 1:
            self.motion.sim_Progress(10)
        
    def weight_lifting(self, pressed_button):
        self.motion.with_Vision = False
        def walk_straight(number_Of_Cycles = 0, stepLength = 0, sideLength = 0, respect_body_tilt = False):
            self.motion.walk_Initial_Pose()
            number_Of_Cycles += 2
            for cycle in range(number_Of_Cycles):
                stepLength1 = stepLength
                if cycle ==0 or cycle == number_Of_Cycles-1 : stepLength1 = stepLength/3
                if cycle ==1 or cycle == number_Of_Cycles-2 : stepLength1 = stepLength/3 * 2
                self.motion.refresh_Orientation()
                #self.motion.body_euler_angle_calc()
                rotation = - self.motion.body_euler_angle['yaw'] * 1.0
                rotation = self.motion.normalize_rotation(rotation)
                self.motion.walk_Cycle(stepLength1, sideLength, rotation,cycle, number_Of_Cycles)
            self.motion.walk_Final_Pose(respect_body_tilt = respect_body_tilt)

        def walk_straight_slow(number_Of_Cycles = 0, stepLength = 0, sideLength = 0):
            amplitude = self.motion.amplitude
            fr1 = self.motion.fr1
            fr2 = self.motion.fr2
            self.motion.amplitude = 70 
            self.motion.fr1 = 50 
            self.motion.fr2 = 20 
            self.motion.walk_Initial_Pose()
            number_Of_Cycles += 2
            for cycle in range(number_Of_Cycles):
                stepLength1 = stepLength
                if cycle ==0 or cycle == number_Of_Cycles-1 : stepLength1 = stepLength/3
                if cycle ==1 or cycle == number_Of_Cycles-2 : stepLength1 = stepLength/3 * 2
                self.motion.refresh_Orientation()
                self.motion.body_euler_angle_calc()
                rotation = - self.motion.body_euler_angle['yaw'] * 1.2
                rotation = self.motion.normalize_rotation(rotation)
                self.motion.walk_Cycle_slow(stepLength1, sideLength, rotation,cycle, number_Of_Cycles)
            self.motion.walk_Final_Pose()
            self.motion.amplitude = amplitude 
            self.motion.fr1 = fr1 
            self.motion.fr2 = fr2 

        if pressed_button == 'start':  
            #walk_straight(number_Of_Cycles = 9, stepLength = 32)
            walk_straight(number_Of_Cycles = self.motion.params['WEIGHTLIFTING_INITIAL_STEPS_NUMBER'],
                       stepLength = self.motion.params['WEIGHTLIFTING_INITIAL_STEPLENGTH'])
            self.motion.jump_turn(0)

        self.motion.play_Soft_Motion_Slot(name = 'Shtanga_1')   

        self.motion.keep_hands_up = True
        self.motion.ztr0 = - 180
        self.motion.ztl0 = - 180
        self.motion.zt0 = - 180
        self.motion.gaitHeight = 170
        self.motion.params['BODY_TILT_AT_WALK'] += self.motion.params['WEIGHTLIFTING_NEXT_BODYTILT']  #22222222222222222222222222222222222
        self.motion.first_Leg_Is_Right_Leg = True
        self.motion.stepHeight = 20
        #walk_straight(number_Of_Cycles = 16, stepLength = 30, respect_body_tilt = True)
        walk_straight(number_Of_Cycles = self.motion.params['WEIGHTLIFTING_NEXT_STEPS_NUMBER'],
                       stepLength = self.motion.params['WEIGHTLIFTING_NEXT_STEPLENGTH'])
        self.motion.params['BODY_TILT_AT_WALK'] -= self.motion.params['WEIGHTLIFTING_NEXT_BODYTILT']

        self.motion.play_Soft_Motion_Slot(name = 'Shtanga_2')          # Weight_Lift_2-2023
        self.motion.keep_hands_up = True
        self.motion.ztr0 = - 170
        self.motion.ztl0 = - 170
        self.motion.zt0 = - 170
        self.motion.params['BODY_TILT_AT_WALK'] += self.motion.params['WEIGHTLIFTING_LAST_BODYTILT']  #333333333333333333333333333333333333333
        #if self.glob.SIMULATION != 5 :  self.motion.params['BODY_TILT_AT_WALK'] = 0
        self.motion.stepHeight = self.motion.params['WEIGHTLIFTING_LAST_STEPHEIGHT']
        walk_straight(number_Of_Cycles = self.motion.params['WEIGHTLIFTING_LAST_STEPS_NUMBER'],
                       stepLength = self.motion.params['WEIGHTLIFTING_LAST_STEPLENGTH'])
        return
    
    def triple_jump_main_cycle(self):
        self.glob.stm_channel.mb.SetBodyQueuePeriod(15)
        time.sleep(10)
        self.motion.play_Soft_Motion_Slot(name = 'TripleJumpForFIRA2023')

    def marathon_main_cycle(self):
        self.motion.params['MARATHON_STEP_LENGTH'] = 0
        self.motion.params['MARATHON_GAIT_HEIGHT'] = 195
        self.motion.params['MARATHON_STEP_HEIGHT'] = 40
        self.motion.params['MARATHON_FPS'] = 4
        self.motion.params['MARATHON_UGOL_TORSA'] = 0
        self.motion.params['MARATHON_BODY_TILT_AT_WALK'] = 0.04
        if self.glob.SIMULATION == 5:
            from Soccer.Vision import lookAtLine

            # Pipeline variables
            turn_shift = Value('i', 0)       #  0 - no order, 1 - straight forward, 2 - to left, 3- to right, 4 - reverse back
                                             #  0X - no shift, 2X - shift to left, 3X - shift to right

            # Process for Vision Pipeline
            cam_proc = Process(target=lookAtLine.camera_process, args=(turn_shift), daemon = True)
            # start Process of Vision Pipeline
            cam_proc.start()

            var = roki2met.roki2met.marathon
            intercom = self.glob.stm_channel.zubr       # used for communication between head and zubr-controller with memIGet/memISet commands
            self.glob.rcb.motionPlay(24)

            while True:
                # wait until motion paramenetrs initiated 
                while True:
                    ok, restart_flag = intercom.memIGet(var.restart_flag)
                    if ok: print('restart_flag :', restart_flag)
                    else: print(intercom.GetError())
                    if restart_flag == 0: break
                    time.sleep(0.25)
            
                # write motion parameters to zubr-controller motion
                intercom.memISet(var.orderFromHead, 0)              #  0 - no order, 1 - straight forward, 2 - to left, 3- to right, 4 - reverse back
                                                                    #  0X - no shift, 2X - shift to left, 3X - shift to right
                intercom.memISet(var.cycle_number, 20000)
                intercom.memISet(var.hipTilt, 0)
                intercom.memISet(var.fps, self.motion.params['MARATHON_FPS'])
                intercom.memISet(var.stepLengthOrder, self.motion.params['MARATHON_STEP_LENGTH'])
                intercom.memISet(var.gaitHeight, self.motion.params['MARATHON_GAIT_HEIGHT'])
                intercom.memISet(var.stepHeight, self.motion.params['MARATHON_STEP_HEIGHT'])
                intercom.memFSet(var.ugol_torsa, self.motion.params['MARATHON_UGOL_TORSA'])
                intercom.memFSet(var.bodyTiltAtWalk, self.motion.params['MARATHON_BODY_TILT_AT_WALK'])
                intercom.memISet(var.pitStop, 1)                    # 1 - go on, 0 - stop waiting

                while True:
                    ok, restart_flag = intercom.memIGet(var.restart_flag)
                    if ok: print('restart_flag :', restart_flag)
                    else: print(intercom.GetError())
                    if restart_flag == 1: break
                    time.sleep(0.25)
                while True:
                    ok, restart_flag = intercom.memIGet(var.restart_flag)
                    if restart_flag == 0: break
                    intercom.memISet(var.orderFromHead, turn_shift.value)
                    time.sleep(0.05)
        # simulation:

        # Pipeline variables
        turn_shift = Value('i', 0)       #  0 - no order, 1 - straight forward, 2 - to left, 3- to right, 4 - reverse back
                                            #  0X - no shift, 2X - shift to left, 3X - shift to right
        """
        stop_flag = Value('i', 0)
        # Process for Vision Pipeline
        cam_proc = Process(target= self.glob.vision.detect_Line_Follow_Stream, args=(turn_shift, stop_flag), daemon = True)
        # start Process of Vision Pipeline
        cam_proc.start()
        """
        event = threading.Event()
        camera_thread = threading.Thread(target = self.glob.vision.detect_Line_Follow_Stream, args=(event, turn_shift))
        camera_thread.setDaemon(True)
        camera_thread.start()

        self.motion.head_Return(0, self.motion.neck_play_pose)
        stepLength = 50
        self.motion.gaitHeight = 190
        number_Of_Cycles = 100
        self.motion.amplitude = 32
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        direction = 0
        for cycle in range(number_Of_Cycles):
            order_from_Head = self.glob.vision.turn_shift
            print('order_from_Head: ', order_from_Head)
            shift = order_from_Head // 10
            turn = order_from_Head % 10
            if shift == 2 : sideLength = -20
            elif shift == 3 : sideLength = 20
            else: sideLength = 0
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            
            if turn == 2: direction += 0.2
            elif turn == 3: direction -= 0.2
            #elif turn == 1: rotation = 0
            rotation = direction + invert * self.motion.imu_body_yaw() * 1.1
            #if rotation > 0: rotation *= 1.5
            rotation = self.motion.normalize_rotation(rotation)

            #rotation = 0
            self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()
        event.set()

    def test_walk_main_cycle(self):
        self.motion.fr1 = 40 #40 #50
        self.motion.fr2 = 20 #12 #20
        self.motion.amplitude = 110 #110
        stepLength = 64
        self.motion.gaitHeight = 210
        #self.motion.stepHeight = 40
        number_Of_Cycles = 10
        #self.motion.amplitude = 32
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        self.motion.walk_Initial_Pose()
        number_Of_Cycles += 1
        for cycle in range(number_Of_Cycles):
            stepLength1 = stepLength
            if cycle ==0 : stepLength1 = stepLength/3
            if cycle ==1 : stepLength1 = stepLength/3 * 2
            self.motion.refresh_Orientation()
            rotation = 0 + invert * self.motion.imu_body_yaw() * 1.1
            #if rotation > 0: rotation *= 1.5
            rotation = self.motion.normalize_rotation(rotation)
            #rotation = 0
            self.motion.walk_Cycle_slow(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
        self.motion.walk_Final_Pose()
        



if __name__=="__main__":
    pass