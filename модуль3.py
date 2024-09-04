import time
import cv2
import multiprocessing
from multiprocessing import Array, Process, Value
from multiprocessing.managers import BaseManager
import numpy as np
frame = Array('B', 1228800)
trigger = Value('i', 0)
sh_arr = np.zeros((650,800,3), np.uint8)
class CustomManager(BaseManager):
    # nothing
    pass
CustomManager.register('shared_array', sh_arr)
manager = CustomManager()
def display(frame, trigger):
    while True:
        if trigger.value == 1:
            #image1 = np.frombuffer(frame.get_obj(), dtype=np.uint8)
            #image1 = image1.reshape(640,640,3)
            trigger.value = 2
            #cv2.imshow("Camera2", image1)
            #cv2.waitKey(10)

        time.sleep(0.01)

if __name__ == '__main__':
    multiprocessing.freeze_support()
    #data_proxy = manager.shared_array()
    process = Process(target=display, args=(frame, trigger,))
    process.start()
    #process.join()
    image = cv2.imread("C:\\Users\\a\\Documents\\Neural\\output_frames\\frame_00000.jpg")
    start = time.perf_counter()
    #frame[:] = image.reshape(1228800)
    trigger.value = 1
    while True:
        if trigger.value == 2: 
            break
        time.sleep(0.01)
    time1 = time.perf_counter() - start
    print("transfer time : ", time1)
    cv2.imshow("Camera1", image)
    cv2.waitKey(0)