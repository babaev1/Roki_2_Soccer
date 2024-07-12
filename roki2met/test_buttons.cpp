/*
  Проект "Roki-2 FIRA"
  Автор
    Зубр ОВК
  Описание
    Тест кнопок на рюкзаке со сменой адресации
*/
#include <roki2met.h>


int restart_flag;
int orderFromHead; // 0 - no order, 1 - straight forward, 2 - to left, 3- to right, 4 - reverse back.
int pitStop;
int startStop;
int head_amp;

void setup() {
  head_amp = 100;
  }
  
void action_at_drop(){
  int frameCount = 80;
  sfPoseGroup(MASK_HEAD_ROTATE, 300, frameCount );
  sfWaitFrame( frameCount );
  sfPoseGroup(MASK_HEAD_ROTATE, -300, frameCount);
  sfWaitFrame(frameCount);
  sfPoseGroup(MASK_HEAD_ROTATE, 0, frameCount);
  sfWaitFrame(frameCount);
}

void action() {
    int frameCount = 80;
    sfPoseGroup(MASK_HEAD_TILT, head_amp, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_HEAD_TILT, -head_amp, frameCount);
    sfWaitFrame(frameCount);
}

//Проверить падение
void testDrop() {
  if( sfAbs(svImuAccX) > 3500 || sfAbs(svImuAccZ) > 3500 ) {
    //Произошло падение
    //Подождать 1 сек
    sfWaitFrame(100);
    action_at_drop();
    }
  }

void main() {
    restart_flag = 0;
    int j;
    int frameCount = 80;
    sfPoseGroup(MASK_ALL, 0, frameCount );
    sfWaitFrame( frameCount );
    restart_flag = 0;
    pitStop = 0;
    startStop = 0;
    setup();
    sfWaitFrame(100);                    // waithing 1sec to change parameters
    svButtonRight = SV_SLOT_INACTIVE;
    svButtonLeft = SV_SLOT_INACTIVE;
    sfBip(1, 1);
    while (svButtonPress != SV_BUTTON_RIGHT_PRESS) sfWaitFrame(1); // waiting to press right button
    svButtonRight = SV_SLOT_RESTART_RUN;
    svButtonLeft = SV_SLOT_RELAX;
    restart_flag = 1;
  
    for (j = 0; j < 1000; j++) {
        action();
        testDrop();
    }
  
}

