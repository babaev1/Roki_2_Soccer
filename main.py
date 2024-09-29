from genericpath import isfile
from multiprocessing.dummy import freeze_support
import sys
import os
import platform
import signal
import math
import json
import time
import datetime
import traceback
import subprocess
from rich.traceback import install
install(show_locals=True)
import matplotlib.pyplot as plt
import numpy as np
from Soccer.Localisation.PF.call_par_filter import particle_filter_create_variables_and_launch
import threading


current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')

if platform.system() == 'Linux':
    with open('/sys/firmware/devicetree/base/model') as model:     
        RPi_model = model.read()
    if RPi_model[:12]== "Raspberry Pi":
        SIMULATION = 5                          # will be running on Raspberry Pi
    else:
        # will be running on desktop computer with Linux
        SIMULATION = 1
else:
    # will be running on desktop computer with Windows
    current_work_directory += '/'
    SIMULATION = 1                      # 0 - Simulation without physics, 
                                        # 1 - Simulation synchronous with physics, 
                                        # 3 - Simulation streaming with physics

if __name__ == '__main__':
    freeze_support()
    #try:
    if SIMULATION == 5:
        filename01 = "output.txt"
        while True:
            time.sleep(1)
            print('New process')
            with open(filename01, "a") as f01:
                print(datetime.datetime.now(), file = f01)
                #p01 = subprocess.Popen(['python', 'main_killable_fira.py'], stderr=f01) #, stdout = f01)
                p01 = subprocess.Popen(['python', 'main_killable.py'], stderr=f01) #, stdout = f01)
            message_was_sounded = False
            counter = 0
            while True:
                p01.poll()
                #print('returncode =', p01.returncode)
                if p01.returncode == 1 :
                    if message_was_sounded == False or counter > 100:
                        os.system("espeak -ven-m1 -a200 'Termination with error'")
                        message_was_sounded = True
                        counter = 0
                if p01.returncode == 5 :
                    if message_was_sounded == False or counter > 100:
                        os.system("espeak -ven-m1 -a200 'Process finished'")
                        message_was_sounded = True
                        counter = 0
                if os.path.isfile('/dev/shm/btn2'):
                    os.system("espeak -ven-m1 -a200 'Process re-load'")
                    p01.terminate()
                    if os.path.isfile('/dev/shm/process.txt'):
                        with open('/dev/shm/process.txt', 'r') as process_file:
                            pid = int(process_file.read())
                            try:
                                os.kill(pid, signal.SIGTERM)
                            except Exception:
                                pass
                        os.remove('/dev/shm/process.txt')
                    p01.poll()
                    print('returncode =', p01.returncode)
                    timer1 = time.perf_counter()
                    while(os.path.isfile('/dev/shm/btn2')):
                        time.sleep(0.1)
                        if time.perf_counter()- timer1 > 2:
                            os.system("espeak -ven-m1 -a200 'Exit from program'")
                            sys.exit(0)
                    break
                time.sleep(0.5)
                counter += 1

    else:
        sys.path.append( current_work_directory + 'Soccer/')
        sys.path.append( current_work_directory + 'Soccer/Motion/')
        sys.path.append( current_work_directory + 'Soccer/Vision/')
        sys.path.append( current_work_directory + 'Soccer/Localisation/')
        sys.path.append( current_work_directory + 'Soccer/Localisation/PF/')
        sys.path.append( current_work_directory)
        
        from class_Glob import Glob
        from class_Vision_Sim import Vision_Sim
        from class_Local import *
        from strategy import Player
        from class_Motion_sim import *

        #goalkeeper, forward_v2, run_test, penalty_Shooter, rotation_test, penalty_Goalkeeper, spot_walk, dance, quaternion_test, test_walk, kick_test, marathon
        second_pressed_button = 'start_later'   # side_step_left, side_step_right, short_run, rotation_right, spot_run, start_later
        role = 'marathon'

        particles_number = 1000
        initial_coord = [-0.5, 0, 0]

        glob = Glob(SIMULATION, current_work_directory, particles_number = particles_number)
        glob.pf_coord = initial_coord

        pf_variables = particle_filter_create_variables_and_launch(glob)

        robots_Number = 1
        robot_ID = '/Telo_Roki_2'

        vision = Vision_Sim(glob)
        motion = Motion_sim(glob, vision, robot_Number = robot_ID)
        motion.sim_Start()
        motion.direction_To_Attack = -initial_coord[2]
        motion.activation()
        local = Local(motion, glob, vision, pf_variables, coord_odometry = initial_coord)
        motion.local = local
        local.coordinate_record(odometry = True)
        motion.falling_Flag = 0

        

        glob.role = role
        m = Player(role, second_pressed_button, glob, motion, local)
        m.play_game()

        if os.path.isfile("Export_data.npy"):
            export_data = np.load("Export_data.npy")
            #rng = export_data[0]
            #rng = int(rng)
            #print(export_data.shape)
            rng = np.arange(export_data.shape[1])
            Dummy_HDataX = export_data[1]
            Dummy_HDataY = export_data[2]
            Dummy_HDataZ = export_data[3]
            BallDataX = export_data[4]
            BallDataY = export_data[5]
            BallDataZ = export_data[6]
            fig,ax = plt.subplots(figsize=(10,6))
            plt.plot(rng,Dummy_HDataX, label = 'Body X')
            plt.plot(rng,Dummy_HDataY, label = 'Body Y')
            #plt.plot(rng,Dummy_HDataZ, label = 'Body Z')
            #plt.plot(rng,BallDataX, label = 'Ball X')
            #plt.plot(rng,BallDataY, label = 'Ball Y')
            #plt.plot(rng,BallDataZ, label = 'Ball Z')
            ax.legend(loc='upper left')
            ax.grid(True)
            plt.show()
            os.remove("Export_data.npy")
            print('average timing for 3m : ', 3 / (Dummy_HDataX[-1] - Dummy_HDataX[0]) * len(Dummy_HDataX) * 0.010)



    #except Exception as e:
    #    if SIMULATION == 5:
    #        exc_type, exc_value, exc_traceback = sys.exc_info()
    #        f = open("Error.txt",'w')
    #        #sys.print_exception(e,f)
    #        traceback.print_exception(exc_type, exc_value, exc_traceback,
    #                              limit=None, file=f)
    #        f.close()
    #        #sys.print_exception(e,sys.stdout)
    #        traceback.print_exception(exc_type, exc_value, exc_traceback,
    #                              limit=None, file=sys.stdout)
    #    else:
    #        print(e)
