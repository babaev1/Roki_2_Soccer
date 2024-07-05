import sys
import os
import math
import json
import time
import traceback
from Soccer.Localisation.class_Glob import Glob
from Soccer.Vision.class_Vision_RPI import Vision_RPI
from Soccer.Localisation.class_Local import *
from Soccer.strategy import Player
from Soccer.Motion.class_Motion_real import Motion_real

SIMULATION = 5
current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
current_work_directory += '/'

try:
    from button_test import Button_Test
    labels = [[],
              ['basketball', 'triple_jump', 'weight_lifting',  'kick_test'],
              #['basketball', 'kick_test'],
              [],
              ['sprint', 'marathon', 'forward'],
              ['FIRA_penalty_Shooter', 'penalty_Goalkeeper', 'run_test']]
    os.system("espeak -ven-m1 -a200 'Choose Role'")
    button = Button_Test(labels)
    first_pressed_button = button.wait_for_button_pressing(message ="'Choose role'")

    with open("/home/pi/Desktop/" + "Init_params/Real/Real_landmarks.json", "r") as f:
        landmarks = json.loads(f.read())
    subrole = ' '
    initial_coord = [0.0, 0.0, 0]
    role = first_pressed_button
    if first_pressed_button == 'forward_center':
        initial_coord = [-0.45, 0, 0]
    if first_pressed_button == 'FIRA_penalty_Shooter':
         initial_coord = [0.2, 0, 0]
    if first_pressed_button == 'penalty_Goalkeeper':
        initial_coord = [-landmarks['FIELD_LENGTH'] / 2, 0, 0]

    if role == 'forward' or role == 'FIRA_penalty_Shooter' or role == 'penalty_Goalkeeper' or role == 'basketball':
        glob = Glob(SIMULATION, current_work_directory, particles_number = 100)
        glob.pf_coord = initial_coord
        vision = Vision_RPI(glob)
        motion = Motion_real(glob, vision)
        motion.falling_Flag = 0
        motion.direction_To_Attack = -initial_coord[2]
        motion.activation()
        local = Local(motion, glob, vision, coord_odometry = initial_coord)
        motion.local = local
        local.coordinate_record(odometry = True)
    else:
        glob = Glob(SIMULATION, current_work_directory, particles_number = 100)
        vision = None
        motion = Motion_real(glob, vision)
        local = None
    
    if role == 'run_test':
        labels = [[],['side_step_left', 'rotation_left'], [], ['short_run', 'long_run', 'spot_run', 'head_tilt_calibration'], ['side_step_right', 'rotation_right']]
    elif role == 'kick_test':
        labels = [[],[],[],['regular', 'new_kick'], []]
    elif role == 'basketball':
        labels = [[],[],[],['start', 'throw_test', 'pick_up_test'], []]
        #labels = [[],[],[],['throw_test', 'start'], []]
    elif role == 'sprint':
        labels = [[],[],[],['start_attention'], []]
    else: 
        labels = [[], [], [], ['start', 'start_later'], []]
    if role == 'forward':
        second_pressed_button = 'start'
    else:
        second_pressed_button = motion.push_Button(labels)

    print( role, subrole, ' initial_coord = ', initial_coord)
    player = Player(role, second_pressed_button, glob, motion, local)
    player.play_game()

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    f = open("Error.txt",'w')
    traceback.print_exception(exc_type, exc_value, exc_traceback,
                          limit=None, file=f)
    f.close()
    traceback.print_exception(exc_type, exc_value, exc_traceback,
                          limit=None, file=sys.stdout)

