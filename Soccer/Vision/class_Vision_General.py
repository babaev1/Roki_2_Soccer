import json
import cv2
import math, time
from Soccer.Vision.reload import Image
from scipy.spatial.transform import Rotation
import numpy as np
from Soccer.Vision.ransac_line_segments_simulation import build_line_segments
import threading

class Vision_General:
    def __init__(self, glob):
        self.glob = glob
        self.glob.vision = self
        self.build_line_segments = build_line_segments
        self.pan = 0
        self.pitch = 0
        self.roll = 0
        self.elevation = 0
        self.yaw = 0
        self.image = None
        self.event = threading.Event()
        self.camera_thread = threading.Thread(target = self.detect_Ball_in_Stream, args=(self.event,))
        self.camera_thread.setDaemon(True)
        #self.camera_thread.start()

    def thinning(self, img, rows, columns, iterations_limit):
        return self.zhangSuen(img, rows, columns, iterations_limit)

    def detect_line_segments( self, img, rank_threshold = 30, line_num_limit = 5, upper_lines = False):
        line_segments = self.build_line_segments( img, rank_threshold, line_num_limit, upper_lines,
                        self.glob.params['CAMERA_HORIZONTAL_RESOLUTION'], self.glob.params['CAMERA_VERTICAL_RESOLUTION'])
        return line_segments

    def orange_ball_is_on_green_field(self, blob, img):
        width = self.glob.params['CAMERA_HORIZONTAL_RESOLUTION']
        height = self.glob.params['CAMERA_VERTICAL_RESOLUTION']
        x , y , w , h = blob.rect()  # ball blob
        x1 = x + w                      # x1, y1, w1, h1-right rectangle
        y1 = y
        if x1 + w <= width:
            w1 = w
        else:
            w1 = width - x1
        if x1 == width:
            w1 = 1
            x1 = width-1
        if y1 + 2 * h <= height:
            h1 = 2 * h
        else:
            h1 = height - y1
        if y1 + h == height:
            h1 = h
        y2 = y                         # x2, y2, w2, h2 - left rectangle
        if x - w > 0:
            x2 = x - w
            w2 = w
        else:
            x2 = 0
            w2 = x1 - x2
        if x1 == 0:
            x2 = 0
            w2 = 1
        y2 = y1
        h2 = h1
        x3 = x                          # x3, y3, w3, h3 - bottom rectangle
        y3 = y + h - 1
        w3 = w
        h3 = h1 - h + 1
        blob_p_g = []                     # right blobs
        blob_l_g = []                     # left blobs
        blob_n_g = []                     # bottom blobs
        blob_p_w = []                     # right blobs
        blob_l_w = []                     # left blobs
        blob_n_w = []                     # bottom blobs
        blob_p_g = img.find_blobs([self.TH['green field']['th']],roi = [x1 , y1 , w1 , h1], pixels_threshold=7, area_threshold=7, merge=True)
        blob_l_g = img.find_blobs([self.TH['green field']['th']],roi = [x2 , y1 , w2 , h1], pixels_threshold=7, area_threshold=7, merge=True)
        blob_n_g = img.find_blobs([self.TH['green field']['th']],roi = [x3 , y3 , w3 , h3], pixels_threshold=7, area_threshold=7, merge=True)
        blob_p_w = img.find_blobs([self.TH['white marking']['th']],roi = [x1 , y1 , w1 , h1], pixels_threshold=7, area_threshold=7, merge=True)
        blob_l_w = img.find_blobs([self.TH['white marking']['th']],roi = [x2 , y1 , w2 , h1], pixels_threshold=7, area_threshold=7, merge=True)
        blob_n_w = img.find_blobs([self.TH['white marking']['th']],roi = [x3 , y3 , w3 , h3], pixels_threshold=7, area_threshold=7, merge=True)
        return len(blob_p_g) > 0 or len( blob_l_g ) > 0  or len( blob_n_g ) > 0 or len(blob_p_w) > 0 or len( blob_l_w ) > 0  or len( blob_n_w ) > 0

    def seek_Ball_In_Frame(self, with_Localization = True):
        see_ball = 0
        timer1 = time.perf_counter()
        for number in range (2):
            camera_result, img1, pitch, roll, yaw, pan = self.snapshot()
            if camera_result:
                img = Image(img1)
                #self.display_camera_image(img1, window = 'Original')
                ball_column, ball_row = 0, 0
                for blob in img.find_blobs([self.TH['orange ball']['th']],
                                    pixels_threshold=self.TH['orange ball']['pixel'],
                                    area_threshold=self.TH['orange ball']['area'],
                                    merge=True):
                    if blob.cy() > ball_row:
                        ball_row = blob.cy()
                        ball_column = blob.cx()
                        ball_blob = blob
                        see_ball += 1
                if see_ball:
                    see_ball = 0
                    if self.orange_ball_is_on_green_field(ball_blob, img): 
                        result, self.camera_elevation = self.glob.motion.camera_elevation()
                        if result:
                            self.visible_reaction_ball()
                            img.draw_rectangle(ball_blob.rect())
                            self.display_camera_image(img.img, window = 'Ball')
                            see_ball += 1
                            break
        if with_Localization and number == 0 and camera_result: self.glob.local.read_Localization_marks(img1)
        #print('seek_Ball_In_Frame time:', time.perf_counter()-timer1)
        if see_ball == 0: return False, 0, 0, 0
        else:
            result, course, distance = self.get_course_and_distance_to_ball(ball_column, ball_row)
            if result: return True, course, distance, ball_blob
            else: return False, 0, 0, 0

    def get_course_and_distance_to_ball(self, ball_column, ball_row):                       # returns relative course from body in radians and relative distance in meters
        result, relative_x_on_floor, relative_y_on_floor = self.image_point_to_relative_coord_on_floor(
                                                    ball_column, ball_row, for_ball = True
                                                    )
        if not result:
            return False, 0, 0
        else:
            print('relative coord of ball on floor (x,y)= ', round(relative_x_on_floor), round(relative_y_on_floor))
            distance = math.sqrt(relative_x_on_floor**2 + relative_y_on_floor**2)
            course = math.atan2(relative_y_on_floor, relative_x_on_floor)
            return True, course, distance/1000

    def get_course_and_distance_to_post(self, blob_cx, blob_y_plus_h):   # returns absolute course in radians and relative distance in meters
        result, relative_x_on_floor, relative_y_on_floor = self.image_point_to_relative_coord_on_floor(
                                                    blob_cx, blob_y_plus_h, absolute = True
                                                    )
        if not result:
            return False, 0, 0
        else:
            relative_distance = math.sqrt(relative_x_on_floor**2 + relative_y_on_floor**2) / 1000
            absolute_course = math.atan2(relative_y_on_floor, relative_x_on_floor)
            return True, absolute_course, relative_distance

    def image_point_to_relative_coord_on_floor(self, column, row, for_ball = False, absolute = False):
        if for_ball: triangulation_base  = self.camera_elevation - self.glob.params['DIAMETER_OF_BALL'] / 2
        #print('triangulation_base = ', triangulation_base)
        u, v = self.undistort_points(column, row)
        point_angle_horizontal = (self.undistort_cx -u)/ self.focal_length_horizontal        # argument of math.atan
        point_angle_vertical = (self.undistort_cy -v)/ self.focal_length_vertical            # argument of math.atan
        point_vector = [1, point_angle_horizontal, point_angle_vertical]    # arguments of math.tan
        pitch_rotation = Rotation.from_euler('y', [self.pitch], degrees=False)
        roll_rotation = Rotation.from_euler('x', [self.roll], degrees=False)
        if absolute:
            pan_rotation = Rotation.from_euler('z', [self.yaw], degrees=False)
        else:
            pan_rotation = Rotation.from_euler('z', [self.pan], degrees=False)
        point_vector = pitch_rotation.apply(point_vector)
        point_vector = roll_rotation.apply(point_vector)
        point_vector = pan_rotation.apply(point_vector)
        if point_vector[0,2] >= 0: return False, 0, 0 
        relative_x_on_floor = point_vector[0,0] * triangulation_base / (- point_vector[0,2])
        relative_y_on_floor = point_vector[0,1] * triangulation_base / (- point_vector[0,2])
        return True, relative_x_on_floor, relative_y_on_floor

    def detect_Ball_Speed(self):
        see_ball = 0
        position = []
        for number in range (2):
            camera_result, img1, pitch, roll, yaw, pan = self.snapshot()
            if camera_result:
                img = Image(img1)
                #self.display_camera_image(self.image, window = 'Original')
                ball_column, ball_row = 0, 0
                for blob in img.find_blobs([self.TH['orange ball']['th']],
                                    pixels_threshold=self.TH['orange ball']['pixel'],
                                    area_threshold=self.TH['orange ball']['area'],
                                    merge=True):
                    if blob.cy() > ball_row:
                        ball_row = blob.cy()
                        ball_column = blob.cx()
                        ball_blob = blob
                        see_ball += 1
                if see_ball:
                    see_ball = 0
                    if self.orange_ball_is_on_green_field(ball_blob, img): 
                        result, self.camera_elevation = self.glob.motion.camera_elevation()
                        if result:
                            self.visible_reaction_ball()
                            img.draw_rectangle(ball_blob.rect())
                            self.display_camera_image(img.img, 'Ball')
                            see_ball += 1
                            result, course, distance = self.get_course_and_distance_to_ball(ball_column, ball_row)
                            if result:
                                position.append([course,distance])
        n = len(position)
        if n > 1:
            front_speed = ( position[n-1][1] - position[0][1])/ distance/n
            tangential_speed = ( position[n-1][0] - position[0][0]) * distance/n
            speed = [tangential_speed, front_speed ]
        if see_ball < 1: return False, 0, 0, [0,0]
        elif n < 2: return False, course, distance, [0,0]
        else: return True, course, distance, speed

    def detect_Ball_in_Stream(self, event):
        while True:
            result, course, distance, speed = self.detect_Ball_Speed()
            if result or distance != 0:
                self.glob.motion.refresh_Orientation()
                yaw = self.glob.local.coord_odometry[2] = self.glob.motion.imu_body_yaw()
                course_global_rad = course + yaw
                proforma_ball_coord = [distance * math.cos(course_global_rad) + self.glob.local.coord_odometry[0],
                                      distance * math.sin(course_global_rad) + self.glob.local.coord_odometry[1]]
                self.glob.local.ball_odometry = [self.glob.local.ball_odometry[0] * 0.5 + proforma_ball_coord[0] * 0.5,
                                                 self.glob.local.ball_odometry[1] * 0.5 + proforma_ball_coord[1] * 0.5]
                self.glob.ball_course = course
                self.glob.ball_distance = distance
                self.glob.ball_speed = speed
                self.glob.robot_see_ball = 5
                #self.glob.local.localisation_Complete()
                #course_global_rad = self.glob.ball_course + self.glob.pf_coord[2]
                #proforma_ball_coord = [self.glob.ball_distance * math.cos(course_global_rad) + self.glob.pf_coord[0],
                #                        self.glob.ball_distance * math.sin(course_global_rad) + self.glob.pf_coord[1]]
                #self.glob.ball_coord = [self.glob.ball_coord[0] * 0.8 + proforma_ball_coord[0] * 0.2,
                #                       self.glob.ball_coord[1] * 0.8 + proforma_ball_coord[1] * 0.2]
            else: self.glob.robot_see_ball -= 1
            time.sleep(self.camera_sleep)
            if event.is_set(): break

if __name__=="__main__":
    pass

