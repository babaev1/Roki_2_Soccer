import serial, time
import os
import threading

class Led:
    def __init__(self):
        self.ser = serial.Serial ('/dev/ttyAMA1') #Open named port
        self.ser.baudrate = 115200 
        os.system("raspi-gpio set 4 a4")
        self.blink = threading.Event()
        self.led_thread = threading.Thread(target = self.waithng_for_led_blink, args = (self.blink,))
        self.led_thread.setDaemon(True)
        self.led_thread.start()
        
    def waithng_for_led_blink(self, event):
        while True:
            if event.is_set():
                self.ser.write(b'\x07')#White LED ON
                time.sleep(0.01)
                self.ser.write(b'\x00')
                event.clear()
            time.sleep(0.5)

if __name__=="__main__":
    led = Led();
    for _ in range(20):
        a = input('click:')
        led.blink.set()
    pass


