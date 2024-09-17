
from Soccer.Localisation.PF.ParticleFilter import ParticleFilter, Agent

class Call_Par_Filter:
    def __init__(self, coord, landmarks_filename, particles_number, current_work_directory):
        self.localized = False
        self.pf = ParticleFilter(Agent(coord[0], coord[1], coord[2]), landmarks_filename, particles_number, current_work_directory)

    def update(self, data, other_coord):                                    # data: {'yellow_posts': [[metr, metr, weight],...,[metr,metr, weight]], 'blue_posts': [[metr, metr, weight],...,[metr,metr, weight]]}
        self.robot_position = self.pf.updatePF( data, other_coord)          # other_coord:  [metr,metr,radians] or []
        if self.pf.consistency > 0.5:                                       #{'Line_crosses': [[metr, metr, weight],...,[metr,metr, weight]]}
            self.localized = True                                           #{'lines': [[rho,theta,weight],..]}
        else:
            self.localized = False                                          #{'penalty': [[metr, metr, weight],...,[metr,metr, weight]]}

    def move(self, odometry):               # odometry:  {'shift_x': Metr, 'shift_y': Metr, 'shift_yaw': Radian}
        self.pf.particles_move(odometry)

    def return_coord(self):                 # (metr,metr)
        return self.pf.return_coord()

    def update_coord(self):
        self.pf.update_coord(self.pf.p)




if __name__=="__main__":
    print('This is not main module!')
