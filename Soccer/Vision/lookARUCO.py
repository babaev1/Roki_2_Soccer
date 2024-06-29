from picamera2 import Picamera2
import cv2
import serial, time, math, json
import RPi.GPIO as GPIO
from time import sleep
import numpy as np
#from Soccer.Vision.led_blink import Led
from led_blink import Led

picam2 = Picamera2(camera_num=0)
led = Led()

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
                center_of_marker_to_frame =  400 - int((top_right[0] + top_left[0])/2)
                #print('size = ', top_right[0] - top_left[0], 'center_of_marker_to_frame = ', center_of_marker_to_frame )
                bottom_right = tuple(corner[2].astype(int))
                bottom_left = tuple(corner[3].astype(int))
                center = (int((top_left[0] + bottom_right[0]) / 2), int((top_left[1] + bottom_right[1]) / 2))     #  центр маркера
                #cv2.circle(frame, center, 5, (0, 255, 0), -1)             # центр маркера
                #cv2.putText(frame, str(ids[i][0]), top_left, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) #  ID маркера 
    return frame 

picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1600, 1300)}, lores={"format": 'YUV420', "size": (800, 650)})) 
picam2.start()

a = 0
start_time = time.perf_counter()
cycles = 1000
while a < cycles:
    request = picam2.capture_request()
    im = request.make_array("lores")  
    request.release()
    im = cv2.cvtColor(im, cv2.COLOR_YUV420p2GRAY)
    #im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    #im = im/256
    #im = im.astype(np.uint8)
    #im = np.uint8(im)
    a += 1
    #im = cv2.cvtColor(im,cv2.COLOR_RGB2GRAY)
    im = detect_aruco_markers(im)     # Обнаружение аруко маркеров
    #cv2.imshow("Camera", im)
    #cv2.waitKey(10)
time_elapsed = time.perf_counter() - start_time
print('time elapsed :', time_elapsed)
print('Rate : ', int(cycles/ time_elapsed), ' FPS')
cv2.destroyAllWindows()
