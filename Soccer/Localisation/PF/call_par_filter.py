import time
from Soccer.Localisation.PF.ParticleFilter import ParticleFilter, Agent
#from ParticleFilter import ParticleFilter, Agent
from multiprocessing import Array, Value, Process

class Call_Par_Filter:
    def __init__(self, coord, landmarks_filename, particles_number, current_work_directory, global_vars, pf_odometry, pf_command, pf_return_coord):
        self.pf_odometry = pf_odometry
        self.pf_command = pf_command
        self.pf_return_coord = pf_return_coord
        self.localized = False
        self.pf = ParticleFilter(Agent(coord[0], coord[1], coord[2]), landmarks_filename, particles_number, current_work_directory, global_vars)

    def update(self, data, other_coord):                                    # data: {'yellow_posts': [[metr, metr, weight],...,[metr,metr, weight]], 'blue_posts': [[metr, metr, weight],...,[metr,metr, weight]]}
        self.robot_position = self.pf.updatePF( data, other_coord)          # other_coord:  [metr,metr,radians] or []
        if self.pf.consistency > 0.5:                                       #{'Line_crosses': [[metr, metr, weight],...,[metr,metr, weight]]}
            self.localized = True                                           #{'lines': [[rho,theta,weight],..]}
        else:
            self.localized = False                                          #{'penalty': [[metr, metr, weight],...,[metr,metr, weight]]}

    def move(self, odometry):               # odometry:  {'shift_x': Metr, 'shift_y': Metr, 'shift_yaw': Radian}
        if self.pf_odometry[0] == 1:
            odometry = {'shift_x': self.pf_odometry[1], 'shift_y': self.pf_odometry[2],'shift_yaw': self.pf_odometry[3]}
            self.pf_odometry[0] = 0
        self.pf.particles_move(odometry)

    def return_coord(self):                 # (metr,metr)
        coord = self.pf.return_coord()
        with self.pf_return_coord.get_lock():
            self.pf_return_coord[0] = 1
            self.pf_return_coord[1] = coord[0]
            self.pf_return_coord[2] = coord[1]
            self.pf_return_coord[3] = coord[2]
        return coord

    def update_coord(self):
        self.pf.update_coord(self.pf.p)


def particle_filter_as_process(coord, landmarks_filename, particles_number, current_work_directory, global_vars, pf_odometry, pf_command, pf_return_coord):
    print('particle_filter_as_process started ')
    call_ = Call_Par_Filter(coord, landmarks_filename, particles_number, current_work_directory, global_vars, pf_odometry, pf_command, pf_return_coord)
    resampling_timer = time.perf_counter()
    while True:
        if pf_odometry[0] > 0:
            with pf_odometry.get_lock():
                for i in range(int(pf_odometry[0])):
                    odometry = {'shift_x': pf_odometry[i * 3 + 1], 'shift_y': pf_odometry[i * 3 + 2],'shift_yaw': pf_odometry[i * 3 + 3]}
                    print('PF: odometry to move: ', odometry)
                    call_.pf.particles_move(odometry)
                pf_odometry[0] = 0
        if pf_command.value == 3:
            with pf_command.get_lock():
                pf_command.value = 0
            call_.pf.updatePF( None, None)
            resampling_timer = time.perf_counter()
        if time.perf_counter() - resampling_timer > 5:
            call_.pf.updatePF( None, None)
            resampling_timer = time.perf_counter()
        call_.return_coord()
        time.sleep(0.5)
        #print('tick')

def particle_filter_create_variables_and_launch(glob):
    if not glob.use_particle_filter: return None
    unsorted_posts = Array('f', 61) # [0] is length of data each data takes 3 positions, total 20 data records can be stored
    post1 = Array('f', 61)
    post2 = Array('f', 61)
    post3 = Array('f', 61)
    post4 = Array('f', 61)
    lines = Array('f', 61)
    Line_crosses = Array('f', 61)
    penalty = Array('f', 61)
    coord_for_PF_global = Array('f', 3)
    pf_odometry = Array('f', 31) # [0] is length of data each data takes 3 positions, total 20 data records can be stored 
    pf_command = Value('i', 0)  # 0 - do nothing, 1 - move, 2- return_coord, 3 - update
    pf_return_coord = Array('f', 4) # if [0] == 1: new data added, if [0] == 0: new data readed. This array returns coordinate x,y,yaw from particle filter
    global_vars = [unsorted_posts, post1, post2, post3, post4, lines, Line_crosses, penalty, coord_for_PF_global]
    variables = [global_vars, pf_odometry, pf_command, pf_return_coord]
    p1 = Process(target = particle_filter_as_process, args = (glob.pf_coord, glob.landmarks_filename, glob.particles_number, glob.current_work_directory,
                                                global_vars, pf_odometry, pf_command, pf_return_coord), daemon = True)
   
    p1.start()
    return variables



if __name__=="__main__":
    from multiprocessing import Array, Value, Process
    import time
    import os

    #print('This is not main module!')
    coord_odometry = [-0.5, 0, 0]
    landmarks_filename = "C:/Users/a/Documents/GitHub/Roki_2_Soccer/Init_params/Sim/Sim_landmarks.json"
    particles_number = 1000
    current_work_directory = "C:/Users/a/Documents/GitHub/Roki_2_Soccer/"
    os.chdir(current_work_directory)
    unsorted_posts = Array('f', 61) # [0] is length of data each data takes 3 positions, total 20 data records can be stored
    post1 = Array('f', 61)
    post2 = Array('f', 61)
    post3 = Array('f', 61)
    post4 = Array('f', 61)
    lines = Array('f', 61)
    Line_crosses = Array('f', 61)
    penalty = Array('f', 61)
    coord_for_PF_global = Array('f', 3)
    pf_odometry = Array('f', 4) # if [0] == 1: new data added, if [0] == 0: new data readed 
    pf_command = Value('i', 0)  # 0 - do nothing, 1 - move, 2- return_coord, 3 - update
    pf_return_coord = Array('f', 4) # if [0] == 1: new data added, if [0] == 0: new data readed
    global_vars = [unsorted_posts, post1, post2, post3, post4, lines, Line_crosses, penalty, coord_for_PF_global]
    p1 = Process(target = particle_filter_as_process, args = (coord_odometry, landmarks_filename, particles_number, current_work_directory,
                                              global_vars, pf_odometry, pf_command, pf_return_coord), daemon = True)
    p1.start()
    #particle_filter_as_process(coord_odometry, landmarks_filename, particles_number, current_work_directory, global_vars, pf_odometry, pf_command, pf_return_coord)

    while True:
        time.sleep(1)
    