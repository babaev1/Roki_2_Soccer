from picamera2 import Picamera2
import cv2
import time

class Camera:
    def __init__(self):
        self.frame_number_counter = None
        self.picam2 = None

    class Frame_number_counter:
        def __init__(self):
            self.list_of_Timestamps = []
            self.head_of_Timestamps = 0
            self.last_processed_frame_number = -1
            self.target_frame_duration = 16700
            
        def add_farme_number(self, request):
            if self.target_frame_duration * 0.98 < request.get_metadata()['FrameDuration'] < self.target_frame_duration * 1.02 :
                self.list_of_Timestamps.append(request.get_metadata()['SensorTimestamp'])
                overlap = self.last_processed_frame_number - self.head_of_Timestamps
                if overlap >= 0:
                    self.list_of_Timestamps = self.list_of_Timestamps[(overlap + 1):]
                    self.head_of_Timestamps += overlap + 1
                # if len(self.list_of_Timestamps) > 100:
                #     self.list_of_Timestamps = self.list_of_Timestamps[40:]
                #     self.head_of_Timestamps += 40
                
    def start(self, exposure = None, gain = None, frame_duration_us = 16700):
        self.picam2 = Picamera2(camera_num=0)
        self.frame_number_counter = self.Frame_number_counter()
        self.frame_number_counter.target_frame_duration = frame_duration_us
        self.picam2.pre_callback = self.frame_number_counter.add_farme_number
        #self.picam2.configure(self.picam2.create_still_configuration(main={"format": 'RGB888', "size": (1600, 1300)}, lores={"format": 'YUV420', "size": (800, 650)}))
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1600, 1300)}, lores={"format": 'YUV420', "size": (800, 650)}))
        if exposure != None:
            self.picam2.set_controls({"ExposureTime": exposure})		# exposure limits: 11 ~ 199953
        if gain != None:
            self.picam2.set_controls({"AnalogueGain": gain})
        self.picam2.set_controls({"FrameDurationLimits": (frame_duration_us*2, frame_duration_us*2)})
        self.picam2.start()
        time.sleep(1)
        self.picam2.set_controls({"FrameDurationLimits": (frame_duration_us, frame_duration_us)})
        self.picam2.set_controls({"AwbEnable": False})
        for _ in range(50):
            if len(self.frame_number_counter.list_of_Timestamps) > 0: return True
            time.sleep(0.02)
        return False
        
    def snapshot(self):
        request = self.picam2.capture_request()
        frame_timestamp = request.get_metadata()['SensorTimestamp']
        frame_duration = request.get_metadata()['FrameDuration']
        try:
            frame_number = self.frame_number_counter.list_of_Timestamps.index(frame_timestamp)\
                            + self.frame_number_counter.head_of_Timestamps
            total_number_of_frames = len(self.frame_number_counter.list_of_Timestamps) + self.frame_number_counter.head_of_Timestamps
            self.frame_number_counter.last_processed_frame_number = frame_number
        except Exception:
            frame_number = None
            total_number_of_frames = None
        #print('frame_number: ', frame_number )
        image = request.make_array("lores")  # image from the "main" stream
        request.release()
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
        #image = cv2.cvtColor(image, cv2.COLOR_YUV420p2RGB)
        image = image[0:650,0:800,0:3]
        return image, frame_number, total_number_of_frames, frame_timestamp, frame_duration
    
    def stop(self):
        self.picam2.stop()

if __name__ == '__main__':
    from class_stm_channel import STM_channel
    stm_channel = STM_channel()
    stm_channel.mb.ResetIMUCounter()
    camera_frame_timestamps = []    
    single_quat_timestamps =[]
    camera = Camera()
    quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = None)
    single_quat_timestamps.append(quat[4] + round(quat[5]/1000000000, 3))
    camera.start() #(exposure = 100000)         # exposure limits: 11 ~ 199953
    start_time = time.perf_counter()
    
    cycles = 10
    for _ in range(cycles):
#         quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = None)
#         single_quat_timestamps.append(quat[4] + round(quat[5]/1000000000, 3))
        image, frame_number, total_number_of_frames, frame_timestamp, frame_duration = camera.snapshot()
        #quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = frame_number)
        camera_frame_timestamps.append(round(frame_timestamp/1000000000, 3))
        #print('frame_number: ', frame_number )
#         cv2.imshow("Camera", image)
#         cv2.waitKey(10)

    FRAME_RATE = 3
    FRAME_RATE = int(1000000 // FRAME_RATE)
    camera.picam2.set_controls({"FrameDurationLimits":(FRAME_RATE,FRAME_RATE)})
    time.sleep(2)
    
    for _ in range(cycles):
#         quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = None)
#         single_quat_timestamps.append(quat[4] + round(quat[5]/1000000000, 3))
        image, frame_number, total_number_of_frames, frame_timestamp, frame_duration = camera.snapshot()
        #quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = frame_number)
        camera_frame_timestamps.append(round(frame_timestamp/1000000000, 3))
        #print('frame_number: ', frame_number )
#         cv2.imshow("Camera", image)
#         cv2.waitKey(10)
    time_elapsed = time.perf_counter() - start_time
    camera.stop()
    #print('frame_number: ', frame_number, 'total_number_of_frames: ', total_number_of_frames, 'frame_timestamp: ', frame_timestamp)
    print('Rate : ', int(cycles/ time_elapsed), ' FPS')
    #cv2.destroyAllWindows()
    print('total number of frames: ', len(camera.frame_number_counter.list_of_Timestamps) + camera.frame_number_counter.head_of_Timestamps)
    #print(camera.frame_number_counter.list_of_Timestamps)
    for i in range(300):
        try:
            #print(i)
            quat = stm_channel.read_quaternion_from_imu_in_head(frame_number = i)
        except Exception: break
    print('IMU records in stm: ', i)
#         timestamp = quat[4] + round(quat[5]/1000000000, 3)
#         print(timestamp, i)
        #time.sleep(1)
#     for timestamp in single_quat_timestamps:
#         print('single_quat_timestamps:', timestamp)
#     for timestamp in camera_frame_timestamps:
#         print('camera_frame_timestamps:', timestamp)
        
    