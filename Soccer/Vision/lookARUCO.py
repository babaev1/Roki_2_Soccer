from picamera2 import Picamera2
from libcamera import controls
import cv2
import time
import numpy as np
from Soccer.Vision.led_blink import Led
import math
import yaml
from yaml.loader import SafeLoader
import os

undistortPointMap = np.load("Soccer/Vision/undistortPointMap_x_y.npy")
P_matrix = np.load("Soccer/Vision/Camera_calibration_P.npy")
undistort_cx, undistort_cy = P_matrix[0,2], P_matrix[1,2]
focal_length_horizontal = P_matrix[0,0]
with open("Soccer/Vision/calibration_matrix.yaml", "r")as f:
    data = yaml.load(f, Loader=SafeLoader)
mtx = np.asarray(data['camera_matrix'])
dist = np.asarray(data['dist_coeff'])
markerSizeInCM = 16

def detect_aruco_markers(frame, led, ID):
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)    #  словарь аруко маркеров
    parameters = cv2.aruco.DetectorParameters_create()    # Параметры детектора

    corners_raw, ids_raw, rejected_img_points = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)   # Обнаружение маркеров
    corners = []
    ids = None
    if ids_raw is not None:
        for i in range(len(ids_raw)):
            if ids_raw[i][0] == ID:
                ids = np.array([ID] ,dtype= np.int32)
                corners.append(corners_raw[i])
    rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, mtx, dist)
    try:
        distance = tvec[0][0][2]
    except Exception: distance = 0
    #print(ids)
    if ids is not None:
        # границы обнаруженных маркеров
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # перебор всех обнаруженных маркеры
        #  координаты углов маркера
        led.blink.set()
        #os.system("espeak -ven-m1 -a100 'Yes'")
        corner = corners[0][0]
        top_left = tuple(corner[0].astype(int))
        top_right = tuple(corner[1].astype(int))
        bottom_left = tuple(corner[2].astype(int))
        
        tr_x, tr_y = undistortPointMap[top_right[0] * 2, top_right[1] * 2]
        tl_x, tl_y = undistortPointMap[top_left[0] * 2, top_left[1] * 2]
        size = tr_x - tl_x
        cx = int((top_right[0] + top_left[0])/2)
        cy = int((top_left[1] + bottom_left[1])/2)
        side_shift =  400 - int((top_right[0] + top_left[0])/2)
        u, v = undistortPointMap[cx * 2, cy * 2]
        aruco_angle_horizontal = math.atan((undistort_cx -u)/ focal_length_horizontal)
        #print('size = ', size, 'side_shift = ', side_shift )

    else:
        size = side_shift = aruco_angle_horizontal = 0
    return frame , size, side_shift, aruco_angle_horizontal, distance

def camera_process(size, side_shift, aruco_angle_horizontal, distance, stopFlag, ID):
    picam2 = Picamera2(camera_num=0)
    led = Led()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1600, 1300)}, lores={"format": 'YUV420', "size": (800, 650)})) 
    picam2.set_controls({"AeExposureMode":  controls.AeExposureModeEnum.Short})
    picam2.start()
    picam2.set_controls({"ExposureTime": 1500})
    picam2.set_controls({"AnalogueGain": 8.0})
    time.sleep(0.5)
    print("exposure : ", picam2.capture_metadata()["ExposureTime"])
    print("gain : ", picam2.capture_metadata()["AnalogueGain"])
    count = 0
    start_time = time.perf_counter()
    while not stopFlag.value:
        request = picam2.capture_request()
        im = request.make_array("lores")  
        #time.sleep(100)
        #cv2.waitKey(50)
        request.release()
        #cv2.waitKey(50)
        im = cv2.cvtColor(im, cv2.COLOR_YUV420p2GRAY)
        #cv2.imshow("Camera1", im)
        #cv2.waitKey(10)
        im, size.value, side_shift.value, aruco_angle_horizontal.value, distance.value = detect_aruco_markers(im, led, ID)     # Обнаружение аруко маркеров
        cv2.imshow("Camera", im)
        cv2.waitKey(10)
        count += 1
        if count == 100:
            count = 0
            time_elapsed = time.perf_counter() - start_time
            print('Rate : ', int(100 / time_elapsed), ' FPS')
            start_time = time.perf_counter()
        if size.value > 180 : 
            print('exit')
            stopFlag.value = True
            break
    cv2.destroyAllWindows()

def evaluate_distance(corners):
    markerSizeInCM = 16
    rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, mtx, dist)
    return tvec[2]

if __name__== "__main__":
    from multiprocessing import Value, Process
    from led_blink import Led
    from ctypes import c_bool
    size = Value('i', 0)
    side_shift = Value('i', 0)
    aruco_angle_horizontal = Value('d', 0)
    distance = Value('d', 0)
    stopFlag = Value(c_bool, False)
    ID = 88
    # Process for Vision Pipeline
    cam_proc = Process(target= camera_process, args=(size, side_shift, aruco_angle_horizontal, distance, stopFlag, ID), daemon = True)
    # start Process of Vision Pipeline
    cam_proc.start()
    while not stopFlag.value:
        aruco_size = size.value
        aruco_shift = side_shift.value
        print("aruco_angle_horizontal: ", aruco_angle_horizontal.value)
        print('distance :', distance.value)
        print('size :', aruco_size)
        if aruco_size > 150:
            print('Reverse')
            stopFlag.value = True
        else:
            if aruco_shift > 0:
                print('Go Left')
            elif aruco_shift < 0:
                print('Go Right')
            else:
                print('Go Straight')
        time.sleep(1)
