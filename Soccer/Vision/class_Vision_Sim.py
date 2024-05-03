import json
import math
import sys
import traceback
from tkinter import EXCEPTION
#import reload
from class_Vision_General import Vision_General

class Vision_Sim(Vision_General):
    def __init__(self, glob):
        super().__init__(glob)
        with open(self.glob.current_work_directory + "Init_params/Sim/Sim_Thresholds.json", "r") as f:
            self.TH = json.loads(f.read())
        self.undistort_cx, self.undistort_cy = self.glob.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2, self.glob.params['CAMERA_VERTICAL_RESOLUTION'] / 2
        self.focal_length_horizontal = self.glob.params['CAMERA_HORIZONTAL_RESOLUTION'] / 2 / math.tan(math.radians(self.glob.params['CAMERA_APERTURE'] / 2))
        self.focal_length_vertical = self.glob.params['CAMERA_VERTICAL_RESOLUTION'] / 2 / math.tan(math.radians(self.glob.params['CAMERA_APERTURE_VERTICAL'] / 2))
        self.camera_sleep = 6

    def snapshot(self):
        try:
            #self.glob.motion.sim_simxSynchronousTrigger(self.glob.motion.clientID)
            self.image = self.glob.motion.vision_Sensor_Get_Image()
            self.glob.motion.refresh_Orientation()
            #print("snapshot: self.glob.motion.euler_angle['pitch'] :", self.glob.motion.euler_angle['pitch'], ' head_tilt: ', self.glob.motion.neck_tilt * self.glob.motion.TIK2RAD)
            print("snapshot: self.glob.motion.euler_angle['roll'] :", self.glob.motion.euler_angle['roll'])
            self.pan = - self.glob.motion.neck_pan * self.glob.motion.TIK2RAD
            self.pitch, self.roll, self.yaw = self.glob.motion.euler_angle['pitch'], self.glob.motion.euler_angle['roll'], self.glob.motion.euler_angle['yaw']
            return True, self.image, self.pitch, self.roll, self.yaw, self.pan
        except Exception: 
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback,
                              limit=None, file=sys.stdout)
            return False, 0,0,0,0,0

    def display_camera_image(self, image, window = 'Vision Sensor'):
        self.glob.motion.vision_Sensor_Display(image, window)

    def visible_reaction_ball(self):
        print('I see ball')

    def undistort_points(self, column, row):
        return column, row

if __name__=="__main__":
    pass

