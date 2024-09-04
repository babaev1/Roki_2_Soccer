from genericpath import isfile
import sys
import os
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


current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')

try:
    with open('/sys/firmware/devicetree/base/model') as model:     
        RPi_model = model.read()
    if RPi_model[:12]== "Raspberry Pi":
        SIMULATION = 5                          # will be running on Raspberry Pi
    else:
        # will be running on desktop computer
        current_work_directory += '/'
        import threading
        SIMULATION = 1
except Exception:
    # will be running on desktop computer
    current_work_directory += '/'
    import threading
    SIMULATION = 1                      # 0 - Simulation without physics, 
                                        # 1 - Simulation synchronous with physics, 
                                        # 3 - Simulation streaming with physics

try:
    if SIMULATION == 5:
        filename01 = "output.txt"
        while True:
            time.sleep(1)
            print('New process')
            with open(filename01, "a") as f01:
                print(datetime.datetime.now(), file = f01)
                p01 = subprocess.Popen(['python', 'main_killable_fira.py'], stderr=f01) #, stdout = f01)
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

        clientID = []
        transfer_Datas =[]
        motion =[]
        local =[]
        glob = []
        vision = []
        robots_Number = 1
        robot_IDs = [ '/Telo_Roki_2', '/Telo_Roki_2[1]', '#1', '#2']
        initial_coord = [[-0.5, 0, 0], [ -1.8, 0, 0], [-1.1, 0, 0], [-1.8, 0, 0]]
        clientID.append(sim_Enable('127.0.0.1', -19997))
        print('clientID1 =', clientID[0])
        if robots_Number > 1:
            clientID.append(sim_Enable('127.0.0.2', -19998))
            print('clientID2 =', clientID[1])
        if robots_Number > 2:
            clientID.append(sim_Enable('127.0.0.3', -19999))
            print('clientID3 =', clientID[2])
        if robots_Number > 3:
            clientID.append(sim_Enable('127.0.0.4', -20000))
            print('clientID4 =', clientID[3])

        events = []
        t = []
        m =[]
        lock = threading.Lock()
        transfer_Data = Transfer_Data()
        for i in range(robots_Number):
            glob.append(Glob(SIMULATION, current_work_directory, particles_number = 1000))
            glob[i].pf_coord = initial_coord[i]
            events.append(threading.Event())
            transfer_Datas.append(transfer_Data)
            vision.append(Vision_Sim(glob[i]))
            motion.append(Motion_sim(glob[i], vision[i], clientID[i] , events[i], lock, transfer_Datas[i], robots_Number, robot_Number = robot_IDs[i]))
            motion[i].sim_Start()
            motion[i].direction_To_Attack = -initial_coord[i][2]
            motion[i].activation()
            local.append(Local(motion[i], glob[i], vision[i], coord_odometry = initial_coord[i]))
            motion[i].local = local[i]
            local[i].coordinate_record(odometry = True)
            motion[i].falling_Flag = 0
            #goalkeeper, forward_v2, run_test, penalty_Shooter, rotation_test, penalty_Goalkeeper, spot_walk, dance, quaternion_test, test_walk, kick_test
            if i == 0 or i == 3:
                second_pressed_button = 'start'   # side_step_left, side_step_right, short_run, rotation_right, spot_run, start_later
                m.append(Player('forward', second_pressed_button, glob[i], motion[i], local[i])) 
            if i == 1:
                second_pressed_button = 'start'
                m.append(Player('forward', second_pressed_button, glob[i], motion[i], local[i]))
            if i == 2:
                second_pressed_button = 'start'
                m.append(Player('forward_v2', second_pressed_button, glob[i], motion[i], local[i]))
            
        if  SIMULATION == 1 :
            t0 = threading.Thread( target = simulation_Trigger_Accumulator, args=(clientID, events, transfer_Datas, lock))
        for i in range(robots_Number):
            t.append(threading.Thread( target = m[i].play_game, args=()))
        if  SIMULATION == 1 :
            t0.setDaemon(True)
            t0.start()
        for i in range(robots_Number): t[i].start()
        while True:
            n = threading.activeCount()
            if n == 2: break
            time.sleep(1)

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



except Exception as e:
    if SIMULATION == 5:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        f = open("Error.txt",'w')
        #sys.print_exception(e,f)
        traceback.print_exception(exc_type, exc_value, exc_traceback,
                              limit=None, file=f)
        f.close()
        #sys.print_exception(e,sys.stdout)
        traceback.print_exception(exc_type, exc_value, exc_traceback,
                              limit=None, file=sys.stdout)
    else:
        print(e)
