from picamera2 import Picamera2
import cv2
import time
import numpy as np
#from Soccer.Vision.led_blink import Led
from led_blink import Led

def detect_aruco_markers(frame):
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)    #  словарь аруко маркеров
    parameters = cv2.aruco.DetectorParameters_create()    # Параметры детектора

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)   # Обнаружение маркеров
    
    #print(ids)
    
    if ids is not None:
        # границы обнаруженных маркеров
        #frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # перебор всех обнаруженных маркеры
        for i in range(len(ids)):
            #  координаты углов маркера
            if ids[i][0] == 88:
                led.blink.set()
                corner = corners[i][0]
                top_left = tuple(corner[0].astype(int))
                top_right = tuple(corner[1].astype(int))
                size = top_right[0] - top_left[0]
                side_shift =  400 - int((top_right[0] + top_left[0])/2)
                #print('size = ', size, 'side_shift = ', side_shift )
    return frame , size, side_shift

def camera_process(size, side_shift, stopFlag):
    picam2 = Picamera2(camera_num=0)
    led = Led()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1600, 1300)}, lores={"format": 'YUV420', "size": (800, 650)})) 
    picam2.start()
    count = 0
    start_time = time.perf_counter()
    while not stopFlag:
        request = picam2.capture_request()
        im = request.make_array("lores")  
        request.release()
        im = cv2.cvtColor(im, cv2.COLOR_YUV420p2GRAY)
        im, size.value, side_shift.value = detect_aruco_markers(im)     # Обнаружение аруко маркеров
        #cv2.imshow("Camera", im)
        #cv2.waitKey(10)
        count += 1
        if count == 100:
            count = 0
            time_elapsed = time.perf_counter() - start_time
            print('Rate : ', int(100 / time_elapsed), ' FPS')
            start_time = time.perf_counter()
        if size > 180 : 
            stopFlag = True
            break
    cv2.destroyAllWindows()
