import json
import cv2
import time, math, json, struct
import reload
from scipy.spatial.transform import Rotation as R
import numpy as np
from camera import Camera

CAMERA_FRAME_DURATION_US = 16700

class Vision:
    def __init__(self, glob):
        self.glob = glob
        if self.glob.SIMULATION == 5: 
            with open(self.glob.current_work_directory + "Init_params/Real/Real_Thresholds.json", "r") as f:
                self.TH = json.loads(f.read)
            self.undistortPointMap = np.load("undistortPointMap_x_y.npy")
            P_matrix = np.load("Camera_calibration_P.npy")
            self.focal_length_horizontal = P_matrix[0,0] #243.54 /2
            self.focal_length_vertical = P_matrix[1,1] #219.9 / 2
            self.camera = Camera()
            self.glob.stm_channel.mb.SetStrobeOffset(5)   # the best value of strobe offset is 9 for 30fps and 5 for 60fps
            self.glob.stm_channel.mb.ConfigureStrobeFilter(CAMERA_FRAME_DURATION_US//1000, 4)
            self.glob.stm_channel.mb.ResetIMUCounter()
            self.camera.start(frame_duration_us=CAMERA_FRAME_DURATION_US)
            #from zhangSuen_Thinning import zhangSuen
#            from ransac_line_segments import build_line_segments
            #self.zhangSuen = zhangSuen
#            self.build_line_segments = build_line_segments

        else:
            with open(self.glob.current_work_directory + "Init_params/Sim/Sim_Thresholds.json", "r") as f:
                self.TH = json.loads(f.read())
            from ransac_line_segments_simulation import build_line_segments
            self.build_line_segments = build_line_segments

    def thinning(self, img, rows, columns, iterations_limit):
        return self.zhangSuen(img, rows, columns, iterations_limit)

    def detect_line_segments( self, img, rank_threshold = 30, line_num_limit = 5, upper_lines = False):
        if self.glob.SIMULATION == 2:
            if upper_lines == True: upperlines = 1
            else: upperlines = 0
            deviation = 2
            number_of_iterations = 10
            raw_lines = img.ransac_segments(rank_threshold, line_num_limit,
                                    upperlines, deviation , number_of_iterations)
            line_segments = []
            for i in range(0,len(raw_lines),4):
                line_segments.append([raw_lines[i],raw_lines[i+1],raw_lines[i+2],raw_lines[i+3]])
        else:
            line_segments = self.build_line_segments( img, rank_threshold, line_num_limit, upper_lines,
                                                    self.glob.params['CAMERA_HORIZONTAL_RESOLUTION'], self.glob.params['CAMERA_VERTICAL_RESOLUTION'])
        return line_segments



if __name__=="__main__":
    v = Vision(1)
    print(v.TH['orange ball'])

