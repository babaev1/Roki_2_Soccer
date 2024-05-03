import time
import os

class Button_Test:
    def __init__(self, list_of_labels):
        self.list_of_labels = list_of_labels
        pass
    def wait_for_button_pressing(self, loudness = '200', message = "'Waiting for button'"):
        os.system("espeak -ven-m1 -a"+ loudness + " " + message)
        counter1 = 0
        period = 200
        result = None
        while True:
            for button in range(len(self.list_of_labels)):
                if os.path.isfile('/dev/shm/btn'+str(button)):
                    for variant in range(len(self.list_of_labels[button])):
                        os.system("espeak -ven-m1 -a"+ loudness + " " + self.list_of_labels[button][variant])
                        result = self.list_of_labels[button][variant]
                        time.sleep(0.5)
                        if os.path.isfile('/dev/shm/btn'+str(button)) != True: break
            if result != None: break
            if counter1 > period:
                period *= 1.5 
                os.system("espeak -ven-m1 -a"+ loudness + " " + message)
                counter1 = 0
            else:
                time.sleep(0.05)
                counter1 += 1
        return result
    
if __name__ == "__main__":
    labels = [[],['one', 'two', 'three'], ['four', 'five', 'six'], ['seven', 'eight', 'nine']]
    b0 = Button_Test(labels)
    print(b0.wait_for_button_pressing())
    
                