from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QApplication, QFileDialog, QTextEdit, QInputDialog, QMessageBox
from PyQt5.QtCore import QThread,QTimer, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QPixmap
import sys
import numpy as np
import cv2
import io 
#import rpc
import serial 
import serial.tools.list_ports
import time
import random
import json
import threading
import os
from picamera2 import Picamera2

from Threshold_Tuner_Layout import *
import Soccer.Vision.reload
from Soccer.Vision.class_Vision_RPI import Vision_RPI as Vision
from Soccer.Localisation.class_Glob import Glob
from Soccer.Vision.reload import Image
from Soccer.Motion.class_Motion_real import Motion_real
from libcamera import controls

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/') + '/'

class Stream(QtCore.QObject):
    """Redirects console output to text widget."""
    newText = QtCore.pyqtSignal(str)
 
    def write(self, text):
        self.newText.emit(str(text))

class VideoThread(QThread):
    #change_pixmap_signal = pyqtSignal(np.ndarray)
    change_pixmap_signal = pyqtSignal()

    def __init__(self,parent=None):
        super().__init__()
        self.parent = parent
        self._run_flag = True

    def run_old(self):
        print('run_')
        # capture from web cam
        cap = cv2.VideoCapture(0)
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                self.change_pixmap_signal.emit(cv_img)
        # shut down capture system
        cap.release()

    def run(self):
        print('run_')
        t = threading.Thread(target=self.parent.camera_monitoring_linux, args=(self,))
        t.setDaemon(True)
        t.start()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()

class mywindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(mywindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.blob_menu_items_list = [self.ui.actionBlobs_detection_OFF, self.ui.actionBlobs_detection_ON, 
                                self.ui.actionOrange_Ball_on_Green_Field, self.ui.actionBlue_Post_on_Green_Field,
                                self.ui.actionYellow_Post_on_Green_Field, self.ui.actionWhite_Post_on_Green_Field]
        self.slider_event1 = threading.Event()
        self.current_device = 'demo'
        self.signal_connection()
        sys.stdout = Stream(newText=self.onUpdateText)
        self.ui.textBrowser_Console.ensureCursorVisible()
        self.ui.comboBox.addItem('demo')
        self.ui.lineEdit_Pixel_TH.setInputMask('999')
        self.ui.lineEdit_Area_TH.setInputMask('999')

        self.image_data = bytearray()
        self.threshold_Dict ={'demo':{"th":[0,100,-127,127,-127,127],"pixel":200,"area":200}, "exposure": 10000, "gain": 1.7}
        self.threshold_file_is_loaded = False
        
        self.connction_stop_event = threading.Event()
        self.connction_stop_event.set()
        self.new_timer = 0
        self.thresholds_are_changing = False
        #self.panel = wx.Panel(self)
        self.filename = '/home/pi/Desktop/Init_params/Real/Real_Thresholds.json'
        with open(current_work_directory + "Threshold_Tuner_config.json", "r") as f:
                self.config = json.loads(f.read())
        self.COM_port = self.config['COM_port']
        self.USB_as_connection = self.config['USB_as_connection']
        self.host_IP = self.config['host_IP']
        self.remote_IP = self.config['remote_IP']
        self.defaultFile = self.config['defaultFile']
        self.threshold_Dict['demo'] = self.config['demo']
        #self.InitUI()
        self.fail_counter = 0
        #img = wx.Image(640, 480)
        self.interface = None
        self.blob_detection = 0
        self.blobs_are_changing = False
        #self.vision
        self.AutoExposure = False
        self.ui.actionAutoExposure_ON.setChecked(False)
        self.ui.actionAutoExposure_OFF.setChecked(True)
        self.exposure = 0
        self.analogie_gain = 0
        self.color_gains =(0,0) 
        self.display_values()
        # create the video capture thread
        self.video_thread = VideoThread(self)
        # connect its signal to the update_image slot
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.bitmap1 = QPixmap()
        self.bitmap2 = QPixmap()
        with open(self.filename, "r") as f:
            loaded_Dict = json.loads(f.read())
            if loaded_Dict.get('orange ball') != None:
                if loaded_Dict.get('blue posts') != None:
                    if loaded_Dict.get('yellow posts') != None:
                        if loaded_Dict.get('white posts') != None:
                            if loaded_Dict.get('green field') != None:
                                if loaded_Dict.get('white marking') != None:
                                    if loaded_Dict.get('line_follow_1') != None:
                                        if loaded_Dict.get('line_follow_2') != None:
                                            if loaded_Dict.get('line_follow_3') != None:
                                                if loaded_Dict.get('line_follow_4') != None:
                                                    self.threshold_file_is_loaded = True
                                                    self.threshold_Dict.update(loaded_Dict)
                                                    #for i in range(12,16,1): self.blobs.Enable(i, True) # enable combined blobs
                                                    self.ui.actionOrange_Ball_on_Green_Field.setEnabled(True)
                                                    self.ui.actionBlue_Post_on_Green_Field.setEnabled(True)
                                                    self.ui.actionYellow_Post_on_Green_Field.setEnabled(True)
                                                    self.ui.actionWhite_Post_on_Green_Field.setEnabled(True)
                                                    self.ui.actionLoad_from_File.setEnabled(False)
                                                    items = list(self.threshold_Dict.keys())
                                                    items.pop(items.index('demo'))
                                                    items.pop(items.index('exposure'))
                                                    items.pop(items.index('gain'))
                                                    self.ui.comboBox.addItems(items)
        print( 'threshold_file_is_loaded =', self.threshold_file_is_loaded)

    def display_values(self):
        self.ui.lineEdit_Pixel_TH.setText(str(self.threshold_Dict[self.current_device]['pixel']))
        self.ui.lineEdit_Area_TH.setText(str(self.threshold_Dict[self.current_device]['area']))
        self.ui.lineEdit_Exposure.setText(str(self.threshold_Dict['exposure']))
        self.ui.lineEdit_Gain.setText(str(self.threshold_Dict['gain']))
        self.ui.horizontalSlider_Lmin.setValue(self.threshold_Dict[self.current_device]['th'][0])
        self.ui.horizontalSlider_Lmax.setValue(self.threshold_Dict[self.current_device]['th'][1])
        self.ui.horizontalSlider_A_min.setValue(self.threshold_Dict[self.current_device]['th'][2])
        self.ui.horizontalSlider_A_max.setValue(self.threshold_Dict[self.current_device]['th'][3])
        self.ui.horizontalSlider_B_min.setValue(self.threshold_Dict[self.current_device]['th'][4])
        self.ui.horizontalSlider_B_max.setValue(self.threshold_Dict[self.current_device]['th'][5])

    def signal_connection(self):
        self.ui.pushButton_Quit.clicked.connect(self.On_Quit_select)
        self.ui.pushButton_Reset_LAB.clicked.connect(self.On_Reset_LAB)
        self.ui.pushButton_SaveExit.clicked.connect(self.On_Save_and_Exit)
        self.ui.actionQuit.triggered.connect(self.On_Quit_select)
        self.ui.pushButton_Load_File.clicked.connect(self.On_Load_File)
        self.ui.actionLoad_from_File.triggered.connect(self.On_Load_File)
        self.ui.actionSave.triggered.connect(self.On_Save)
        self.ui.actionSave_as.triggered.connect(self.On_Save_as)
        self.ui.actionAbout.triggered.connect(self.On_About)
        self.ui.actionQuick_Start.triggered.connect(self.Quick_Start)
        self.ui.actionDefault.triggered.connect(self.On_Deafault_Connection)
        self.ui.actionUSB.triggered.connect(self.On_USB_input)
        self.ui.lineEdit_Pixel_TH.returnPressed.connect(self.On_number_input_pixel)
        self.ui.lineEdit_Area_TH.returnPressed.connect(self.On_number_input_area)
        self.ui.lineEdit_Exposure.returnPressed.connect(self.On_number_input_exposure)
        self.ui.lineEdit_Gain.returnPressed.connect(self.On_number_input_gain)
        self.ui.pushButton_Start_Camera.clicked.connect(self.On_Start_Camera)
        for i in range(len(self.blob_menu_items_list)):
            self.blob_menu_items_list[i].triggered.connect(self.On_Blobs_Change)
        self.ui.actionAutoExposure_OFF.triggered.connect(self.On_AutoExposure_OFF)
        self.ui.actionAutoExposure_ON.triggered.connect(self.On_AutoExposure_ON)   
        self.ui.horizontalSlider_Lmin.sliderMoved.connect(self.On_Slider_move)
        self.ui.horizontalSlider_Lmax.sliderMoved.connect(self.On_Slider_move)
        self.ui.horizontalSlider_A_min.sliderMoved.connect(self.On_Slider_move)
        self.ui.horizontalSlider_A_max.sliderMoved.connect(self.On_Slider_move)
        self.ui.horizontalSlider_B_min.sliderMoved.connect(self.On_Slider_move)
        self.ui.horizontalSlider_B_max.sliderMoved.connect(self.On_Slider_move)
        self.ui.comboBox.currentIndexChanged.connect(self.On_device_selector)

    def On_AutoExposure_OFF(self,event):
        try:
            self.vision.camera.picam2.set_controls({"AeEnable": False})
            print("AutoExposure OFF")
            self.ui.actionAutoExposure_ON.setChecked(False)
            self.AutoExposure = False
        except Exception:
            print('Failed AutoExposure Off. Camera is not started')

    def On_AutoExposure_ON(self,event):
        try:
            self.vision.camera.picam2.set_controls({"AeEnable": True})
            self.vision.camera.picam2.set_controls({"AeExposureMode":  controls.AeExposureModeEnum.Short})
            time.sleep(1)
            print("AutoExposure ON")
            self.ui.actionAutoExposure_OFF.setChecked(False)
            self.threshold_Dict['exposure'] = self.vision.camera.picam2.capture_metadata()["ExposureTime"]
            self.ui.lineEdit_Exposure.setText(str(self.threshold_Dict['exposure']))
            self.threshold_Dict['gain'] = self.vision.camera.picam2.capture_metadata()["AnalogueGain"]
            self.ui.lineEdit_Gain.setText(str(self.threshold_Dict['gain']))
            self.AutoExposure = True
        except Exception:
            print('Failed AutoExposure ON. Camera is not started')

    def onUpdateText(self, text):
        """Write console output to text widget."""
        cursor = self.ui.textBrowser_Console.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        cursor.insertText(text)
        self.ui.textBrowser_Console.setTextCursor(cursor)
        self.ui.textBrowser_Console.ensureCursorVisible()

    def Quick_Start(self, event):
        print('1. Verify that remote part of this software is launched on OpenMV cam',
              '\n2. Load thresholds from file by pressing button <Load File> or through menu item <File>/<Load from file>.',
              ' Note that it is not allowed to load file 2 times in single session.',
              ' You have to restart program if you need to tune thresholds in different file.',
              '\n3. Choose communication channel through menu item <File>/<Connection Port> and set up settings or just use <Default>',
              ' if you have it done last time.',
              '\n4. Press button <Start Camera>',
              '\n5. Choose color device from dropdown menu under "demo" and tune thresholds by sliders',
              '\n6. Note that moving sliders will affect to changing of thresholds after sliders stops moving'
              '\n7. press <Reset LAB> if you wish to lead all sliders quickly to position with maximum gaps',
              '\n8. Blobs detection mode can be chosen under <Blobs> menu',
              '\n9. <Pixel TH> and <Area TH> settings affect to detection of blobs.',
              ' With bigger values in these settings less blobs can be detected',
              '\n10. Save changes into loaded thresholds file or save in new file by designation new filename in menu item <File>/<Save as>',
              '\n11. Press <Quit> button if you wish to exit without saving.',
              '\n USB settings, WiFi setting, default file name and current slider positions for "demo" device',
              ' will be stored in configuration file for next session')

    def On_About(self, event):
        msgBox = QMessageBox()
        msgBox.setText(" Threshold Tuner\n Version 2.0\n With this app you can tune fast and convenient color thresholds for OpenMV camera.\n\
                        (C) 2022\n www.robokit.su\n Azer Babaev")
        msgBox.exec()

    def On_Deafault_Connection(self, event):
        with open(current_work_directory + "Threshold_Tuner_config.json", "r") as f:
                config = json.loads(f.read())
        self.COM_port = config['COM_port']
        self.USB_as_connection = config['USB_as_connection']
        self.host_IP = config['host_IP']
        self.remote_IP = config['remote_IP']

    def On_USB_input(self, event):
        choices = []
        message = 'List of available ports. Choose COM port'
        for port, desc, hwid in serial.tools.list_ports.comports():
            message = message + '\n' + str(port) + ' - ' + str(desc)
            choices.append(str(port))
        COM_port, okPressed =  QInputDialog.getItem(self.ui.centralwidget, message,"COM ports:", choices, 0, False)
        if COM_port and okPressed:
            self.COM_port = COM_port
            self.USB_as_connection = True
            print('COM_port = ', self.COM_port )

    def On_number_input_pixel(self):
        self.threshold_Dict[self.current_device]['pixel'] = int(self.ui.lineEdit_Pixel_TH.text())
        print('pixel :', self.threshold_Dict[self.current_device]['pixel'])
        self.ui.lineEdit_Pixel_TH.setText(str(self.threshold_Dict[self.current_device]['pixel']))
        self.blobs_are_changing = True
        self.slider_event1.set()

    def On_number_input_area(self):
        self.threshold_Dict[self.current_device]['area'] = int(self.ui.lineEdit_Area_TH.text())
        print('area :', self.threshold_Dict[self.current_device]['area'])
        self.ui.lineEdit_Area_TH.setText(str(self.threshold_Dict[self.current_device]['area']))
        self.blobs_are_changing = True
        self.slider_event1.set()

    def On_number_input_exposure(self):
        if not self.AutoExposure:
            new_exposure = min(int(self.ui.lineEdit_Exposure.text()), 16500)
            self.threshold_Dict['exposure'] = new_exposure
            self.ui.lineEdit_Exposure.setText(str(self.threshold_Dict['exposure']))
            print('Exposure :', self.threshold_Dict['exposure'])
            self.vision.camera.picam2.set_controls({"ExposureTime": self.threshold_Dict['exposure']})
            self.slider_event1.set()
        else:
            self.threshold_Dict['exposure'] = self.vision.camera.picam2.capture_metadata()["ExposureTime"]
            self.ui.lineEdit_Exposure.setText(str(self.threshold_Dict['exposure']))
            print('AutoExposure is ON')

    def On_number_input_gain(self):
        if not self.AutoExposure:
            self.threshold_Dict['gain'] = float(self.ui.lineEdit_Gain.text())
            print('Gain :', self.threshold_Dict['gain'])
            self.vision.camera.picam2.set_controls({"AnalogueGain": self.threshold_Dict['gain']})
            self.slider_event1.set()
        else:
            self.threshold_Dict['gain'] = self.vision.camera.picam2.capture_metadata()["AnalogueGain"]
            self.ui.lineEdit_Gain.setText(str(self.threshold_Dict['gain']))
            print('AutoExposure is ON')

    def On_Blobs_Change(self):
        for i in range(len(self.blob_menu_items_list)):
            if self.blob_menu_items_list[i] == self.sender():
                id = i
                self.blob_menu_items_list[i].setChecked(True)
            else:
                self.blob_menu_items_list[i].setChecked(False)
        print('blobs id =', id)
        self.blob_detection = id
        self.blobs_are_changing = True
        self.slider_event1.set()

    def On_Save(self, event):
        data = self.threshold_Dict.copy()
        data.pop('demo')
        with open(self.filename, "w") as f:
                json.dump(data, f)

    def On_Save_as(self, event):
        fileName, wildcard = QFileDialog.getSaveFileName(None, "Save File as .json file", self.filename, '*.json')
        if fileName:
            self.filename = fileName
            data = self.threshold_Dict.copy()
            data.pop('demo')
            with open(self.filename, "w") as f:
                json.dump(data, f)

    def On_Save_and_Exit(self, event):
        self.On_Save(event)
        self.On_Quit_select(event)

    def On_device_selector(self,event):
        value = self.ui.comboBox.currentText()
        if value != self.current_device:
            if value != '':
                self.current_device = value
                print('current device:', self.current_device)
                self.display_values()
                th = self.threshold_Dict[self.current_device]
                self.slider_event1.set()
            else:
                ind = list(self.threshold_Dict.keys()).index(self.current_device)
                self.ui.comboBox.setCurrentText(ind)

    def On_Slider_move(self, e):
        self.slider_event1.set()

    def On_Quit_select(self, e):
        self.config = {'USB_as_connection':self.USB_as_connection, 'COM_port': self.COM_port,
                      'host_IP': self.host_IP, 'remote_IP': self.remote_IP, 'demo': self.threshold_Dict['demo'],
                      'defaultFile': self.defaultFile}
        with open(current_work_directory + "Threshold_Tuner_config.json", "w") as f:
                json.dump(self.config, f)
        self.connction_stop_event.clear()
        self.connction_stop_event.wait(timeout=1)
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        sys.exit(0)

    def On_Reset_LAB(self, e):
        self.threshold_Dict[self.current_device]['th'] = [0,100,-127,127,-127,127]
        self.display_values()
        self.slider_event1.set()

    def On_Load_File(self, event):
        load_file_dialog = QFileDialog.getOpenFileName(None, str("Open Threshold file"), self.defaultFile, str("*.json"))
        print('filename:', bool(load_file_dialog[0]))
        if load_file_dialog[0]:
            self.filename = load_file_dialog[0]
            self.defaultFile = self.filename
            with open(self.filename, "r") as f:
                loaded_Dict = json.loads(f.read())
            if loaded_Dict.get('orange ball') != None:
                if loaded_Dict.get('blue posts') != None:
                    if loaded_Dict.get('yellow posts') != None:
                        if loaded_Dict.get('white posts') != None:
                            if loaded_Dict.get('green field') != None:
                                if loaded_Dict.get('white marking') != None:
                                    if loaded_Dict.get('line_follow_1') != None:
                                        if loaded_Dict.get('line_follow_2') != None:
                                            if loaded_Dict.get('line_follow_3') != None:
                                                if loaded_Dict.get('line_follow_4') != None:
                                                    self.threshold_file_is_loaded = True
                                                    self.threshold_Dict.update(loaded_Dict)
                                                    self.ui.pushButton_Load_File.setDisabled(True)
                                                    self.ui.actionLoad_from_File.setDisabled(True)
                                                    #for i in range(12,16,1): self.blobs.Enable(i, True) # enable combined blobs
                                                    self.ui.actionOrange_Ball_on_Green_Field.setEnabled(True)
                                                    self.ui.actionBlue_Post_on_Green_Field.setEnabled(True)
                                                    self.ui.actionYellow_Post_on_Green_Field.setEnabled(True)
                                                    self.ui.actionWhite_Post_on_Green_Field.setEnabled(True)
                                                    self.ui.comboBox.addItems(list(self.threshold_Dict.keys())[1:])
        print( 'threshold_file_is_loaded =', self.threshold_file_is_loaded)

    def closeEvent(self, event):
        """Shuts down application on close."""
        # Return stdout to defaults.
        sys.stdout = sys.__stdout__
        self.video_thread.stop()
        event.accept()
        super().closeEvent(event)

    @pyqtSlot(np.ndarray)
    def update_image_old(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.ui.label_Color_Frame.setPixmap(qt_img)

    @pyqtSlot()
    def update_image(self):
        """Updates the image_label with a new opencv image"""
        self.ui.label_Color_Frame.setPixmap(self.bitmap2)
        self.ui.label_Binary_Frame.setPixmap(self.bitmap1)

    def convert_cv_qt(self, rgb_image):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
#         rgb_image = rgb_image/256
#         rgb_image = rgb_image.astype(np.uint8)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        #p = convert_to_Qt_format.scaled(480, 380, Qt.KeepAspectRatio)
        p = convert_to_Qt_format.scaled(800, 650, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def On_Start_Camera(self, event):
        print('Start Camera pressed')
        self.fail_counter = 0
        # start the thread
        self.video_thread.start()
        self.ui.pushButton_Start_Camera.setEnabled(False)

    def camera_monitoring_linux(self, parent):
        self.parent = parent
        # picam2 = Picamera2(camera_num=0)
        # picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1600, 1300)}, lores={"format": 'YUV420', "size": (800, 650)}))
        # picam2.start()
        # time.sleep(0.5)
        self.glob = Glob(5, current_work_directory)
        self.vision = Vision(self.glob)
        self.motion = Motion_real(self.glob, self.vision)
        self.vision.TH = self.threshold_Dict
        timer = time.perf_counter()
        while(True):
            if not self.connction_stop_event.is_set():
                self.connction_stop_event.set()
                return
            self.new_timer = 0
            lmin, lmax, amin, amax, bmin, bmax = self.threshold_Dict[self.current_device]['th']
            # request = picam2.capture_request()
            # cv2_image = request.make_array("lores")
            # cv2_image = cv2.cvtColor(cv2_image, cv2.COLOR_YUV2BGR_I420)
            # cv2_image = cv2_image[0:650,0:800,0:3]
            # request.release()
            camera_result, cv2_image, pitch, roll, yaw, pan = self.vision.snapshot()
            if camera_result:
                color_image= Image(cv2_image)
                if self.blob_detection != 0:
                    if self.blob_detection == 1:
                        #print('self.threshold_Dict[self.current_device]:', self.threshold_Dict[self.current_device])
                        for blob in color_image.find_blobs([self.threshold_Dict[self.current_device]['th']],
                                                        pixels_threshold = self.threshold_Dict[self.current_device]['pixel'],
                                                        area_threshold = self.threshold_Dict[self.current_device]['area']):
                            color_image.draw_rectangle(blob.rect(), color = (0, 0, 255))
                            print(blob.rect())
                    if self.blob_detection == 2:
                        see_ball = 0
                        ball_column, ball_row = 0, 0
                        for blob in color_image.find_blobs([self.vision.TH['orange ball']['th']],
                                            pixels_threshold=self.vision.TH['orange ball']['pixel'],
                                            area_threshold=self.vision.TH['orange ball']['area'],
                                            merge=True):
                            if blob.cy() > ball_row:
                                ball_row = blob.cy()
                                ball_column = blob.cx()
                                ball_blob = blob
                                see_ball += 1
                        if see_ball:
                            see_ball = 0
                            if self.vision.orange_ball_is_on_green_field(ball_blob, color_image): 
                                self.vision.visible_reaction_ball()
                                color_image.draw_rectangle(ball_blob.rect(), color = (0, 0, 255))
                cv2_image = cv2.resize(cv2_image, (976,800), interpolation= cv2.INTER_LINEAR)
                reload_image = Image(cv2_image)
                binary_image = reload_image.binary([lmin, lmax, amin, amax, bmin, bmax])
                color_image.img = cv2.resize(color_image.img, (976,800), interpolation= cv2.INTER_LINEAR)
                self.bitmap2 = self.convert_cv_qt(color_image.img)
                self.bitmap1 = self.convert_cv_qt(binary_image)
                self.parent.change_pixmap_signal.emit()
                if not self.connction_stop_event.is_set(): 
                    try:
                        raise ValueError('oops!')
                    except ValueError: time.sleep(1.5)
                if self.slider_event1.is_set():
                    self.new_timer = time.perf_counter()
                    self.slider_event1.clear()
                    self.thresholds_are_changing = True
                if self.thresholds_are_changing or self.blobs_are_changing:
                    if (time.perf_counter() - self.new_timer) > 0.02:
                        self.thresholds_are_changing = False
                        self.blobs_are_changing = False
                        lmin = self.ui.horizontalSlider_Lmin.value()
                        lmax = self.ui.horizontalSlider_Lmax.value()
                        amin = self.ui.horizontalSlider_A_min.value()
                        amax = self.ui.horizontalSlider_A_max.value()
                        bmin = self.ui.horizontalSlider_B_min.value()
                        bmax = self.ui.horizontalSlider_B_max.value()
                        self.threshold_Dict[self.current_device]['th'] = [lmin, lmax, amin, amax, bmin, bmax]
            else:
                if self.glob.camera_down_Flag == True:
                    self.glob.camera_down_Flag = False
                    self.glob.vision.camera.picam2.close()
                    self.glob.vision.event.set()
                    new_stm_channel  = self.glob.STM_channel_class(self.glob)
                    self.glob.stm_channel = new_stm_channel
                    self.glob.rcb = self.glob.stm_channel.rcb
                    new_vision = Vision(self.glob)
                    self.glob.vision = new_vision
                    self.motion.vision = self.glob.vision
                    self.vision = self.glob.vision
                    self.vision.camera.picam2.set_controls({"ExposureTime": self.threshold_Dict['exposure']})
                    self.vision.camera.picam2.set_controls({"AnalogueGain": self.threshold_Dict['gain']})
                    self.vision.TH = self.threshold_Dict
                    self.vision.camera_thread.start()
            #print('FPS:', 1/(time.perf_counter() - timer))
            timer = time.perf_counter()

if __name__=="__main__":
    app =  QApplication(sys.argv)
    print('font:', app.font().pointSize())
    a = mywindow()
    a.resize(1840, 940)
    a.show()
    sys.exit(app.exec_())
