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
from Soccer.Localisation.PF.call_par_filter import particle_filter_create_variables_and_launch

SIMULATION = 5
current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
current_work_directory += '/'

try:
    from button_test import Button_Test
    labels = [[],
              ['basketball', 'triple_jump', 'weight_lifting',  'kick_test'],
              [],
              ['sprint', 'marathon', 'forward', 'dance'],
              ['FIRA_penalty_Shooter', 'penalty_Goalkeeper', 'run_test', 'jump_test']]
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
         initial_coord = [0.8, 0, 0]
    if first_pressed_button == 'penalty_Goalkeeper':
        initial_coord = [-landmarks['FIELD_LENGTH'] / 2, 0, 0]

    if role == 'forward' or role == 'FIRA_penalty_Shooter' or role == 'penalty_Goalkeeper' or\
                    role == 'basketball' or role == 'marathon' or role == 'weight_lifting' or\
                    role == 'triple_jump':   #or role == 'run_test' or role == 'weight_lifting':
        glob = Glob(SIMULATION, current_work_directory, particles_number = 100, event_type = 'FIRA')
        glob.pf_coord = initial_coord
        glob.role = role
        if (role == 'forward' or role == 'FIRA_penalty_Shooter' or role == 'penalty_Goalkeeper'or role == 'weight_lifting' or role == 'basketball'):
            glob.neural_vision_enable()
            glob.neural_vision = True
        vision = Vision_RPI(glob)
        motion = Motion_real(glob, vision)
        motion.falling_Flag = 0
        motion.direction_To_Attack = -initial_coord[2]
        motion.activation()

        pf_variables = particle_filter_create_variables_and_launch(glob)

        local = Local(motion, glob, vision, pf_variables, coord_odometry = initial_coord)
        motion.local = local
        local.coordinate_record(odometry = True)
    else:
        glob = Glob(SIMULATION, current_work_directory, particles_number = 100)
        glob.with_Local = False
        vision = None
        motion = Motion_real(glob, vision)
        local = None
        motion.with_Vision = False
        glob.camera_streaming = False
        motion.activation()
        
    
    if role == 'run_test':
        labels = [[],['side_step_left', 'rotation_left'], [], ['short_run', 'long_run', 'spot_run', 'run_backwards'], ['side_step_right', 'rotation_right']]
    elif role == 'jump_test':
        labels = [[],['jump_left'], [], ['jump_forward', 'jump_backward', 'jump_on_spot'], ['jump_right']]
    elif role == 'kick_test':
        labels = [[],[],[],['regular', 'new_kick'], []]
    elif role == 'basketball':
        labels = [[],[],[],['start', 'start_with_approach', 'throw_control', 'pick_up_test', 'approach_test'], []]
        #labels = [[],[],[],['throw_test', 'start'], []]
    elif role == 'weight_lifting':
        labels = [[],[],[],['start', 'start_simple', 'start_lifting'], []]
    elif role == 'sprint':
        labels = [[],[],[],['start_attention'], []]
    elif role == 'triple_jump':
        labels = [[],[],[],['start', 'start_with_approach', 'find_launch_bar'], []]
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

