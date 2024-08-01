/*
  Проект "Roki-2 FIRA"
  Автор
    Зубр ОВК
  Описание
    Скрипт вставание робота Roki-2 с 2-мя батареями
*/
#include <roki2met.h>

int restart_flag;
int fallingFlag; // = 0

int torsoAdd;
float forwardDirection;
float correctedRotation;
int frameCount;

void standUp() {
    if (fallingFlag == 1) {  // on stomach
        // page 0
        frameCount = 10;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 5145, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 384, frameCount);
        sfWaitFrame(frameCount);
        // page 1
        frameCount = 50;
        sfPoseGroup(MASK_RIGHT_FOOT_FRONT | MASK_LEFT_FOOT_FRONT, 3698, frameCount);
        sfPoseGroup(MASK_RIGHT_KNEE | MASK_LEFT_KNEE, 6144, frameCount);
        sfPoseGroup(MASK_RIGHT_HIP | MASK_LEFT_HIP, 6084, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 5836, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 3087, frameCount);
        sfWaitFrame(frameCount);
        // page 2
        frameCount = 10;
        sfPoseGroup(MASK_RIGHT_FOOT_FRONT | MASK_LEFT_FOOT_FRONT, 3072, frameCount);
        sfPoseGroup(MASK_RIGHT_HIP | MASK_LEFT_HIP, 5222, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 3072, frameCount);
        sfWaitFrame(frameCount);
        // page 3
        frameCount = 10;
        sfPoseGroup(MASK_RIGHT_FOOT_FRONT | MASK_LEFT_FOOT_FRONT, 2267, frameCount);
        sfPoseGroup(MASK_RIGHT_KNEE | MASK_LEFT_KNEE, 6144, frameCount);
        sfPoseGroup(MASK_RIGHT_HIP | MASK_LEFT_HIP, 5130, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 2304, frameCount);
        sfWaitFrame(frameCount);
        // page 4
        frameCount = 50;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 5145, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 384, frameCount);
        sfWaitFrame(frameCount);
    }
    if (fallingFlag == -1) {  // face up
        // page 0
        frameCount = 20;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfWaitFrame(frameCount);
        // page 2
        frameCount = 30;
        sfPoseGroup(MASK_RIGHT_FOOT_FRONT | MASK_LEFT_FOOT_FRONT, 3532, frameCount);
        sfPoseGroup(MASK_RIGHT_KNEE | MASK_LEFT_KNEE, 5376, frameCount);
        sfPoseGroup(MASK_RIGHT_HIP | MASK_LEFT_HIP, -2457, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 2150, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW_SIDE | MASK_LEFT_ELBOW_SIDE, -3072, frameCount);
        sfPoseGroup(MASK_RIGHT_SHOULDER | MASK_LEFT_SHOULDER, 7680, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 2764, frameCount);
        sfWaitFrame(frameCount);
        // page 4 clone 2
        frameCount = 20;
        sfPoseGroup(MASK_RIGHT_KNEE | MASK_LEFT_KNEE, 3717, frameCount);
        sfPoseGroup(MASK_RIGHT_HIP | MASK_LEFT_HIP, -2274, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 4254, frameCount);
        sfPoseGroup(MASK_HEAD_TILT, 4712, frameCount);
        sfWaitFrame(frameCount);
        // page 7
        frameCount = 50;
        sfPoseGroup(MASK_RIGHT_FOOT_SIDE | MASK_LEFT_FOOT_SIDE, 102, frameCount);
        sfPoseGroup(MASK_RIGHT_FOOT_FRONT | MASK_LEFT_FOOT_FRONT, 1721, frameCount);
        sfPoseGroup(MASK_RIGHT_KNEE | MASK_LEFT_KNEE, 3445, frameCount);
        sfPoseGroup(MASK_RIGHT_HIP | MASK_LEFT_HIP, 1721, frameCount);
        sfPoseGroup(MASK_RIGHT_HIP_SIDE | MASK_LEFT_HIP_SIDE, 102, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 4094, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW_SIDE | MASK_LEFT_ELBOW_SIDE, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_SHOULDER | MASK_LEFT_SHOULDER, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 399, frameCount);
        sfPoseGroup(MASK_HEAD_TILT, 0, frameCount);
        sfWaitFrame(frameCount);
        // page 9
        frameCount = 10;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 5145, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 384, frameCount);
        sfWaitFrame(frameCount);
    }
    if (fallingFlag == 2) {  // on left side
        // page 0
        frameCount = 40;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW, 3072, frameCount);
        sfPoseGroup(MASK_RIGHT_SHOULDER, 768, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE, -768, frameCount);
        sfPoseGroup(MASK_LEFT_ELBOW, 3840, frameCount);
        sfPoseGroup(MASK_LEFT_SHOULDER, 3840, frameCount);
        sfPoseGroup(MASK_LEFT_CLAVICLE, 3840, frameCount);
        sfWaitFrame(frameCount);
        // page 2
        frameCount = 60;
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 3072, frameCount);
        sfPoseGroup(MASK_RIGHT_SHOULDER | MASK_LEFT_SHOULDER, 307, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, -1228, frameCount);
        sfWaitFrame(frameCount);
        // page 3
        frameCount = 20;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfWaitFrame(frameCount);
    }
    if (fallingFlag == -2) {  // on right side
        // page 0
        frameCount = 40;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfPoseGroup(MASK_RIGHT_ELBOW, 3840, frameCount);
        sfPoseGroup(MASK_RIGHT_SHOULDER, 3840, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE, 3840, frameCount);
        sfPoseGroup(MASK_LEFT_ELBOW, 3072, frameCount);
        sfPoseGroup(MASK_LEFT_SHOULDER, 768, frameCount);
        sfPoseGroup(MASK_LEFT_CLAVICLE, -768, frameCount);
        sfWaitFrame(frameCount);
        // page 2
        frameCount = 60;
        sfPoseGroup(MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 3072, frameCount);
        sfPoseGroup(MASK_RIGHT_SHOULDER | MASK_LEFT_SHOULDER, 307, frameCount);
        sfPoseGroup(MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, -1228, frameCount);
        sfWaitFrame(frameCount);
        // page 3
        frameCount = 20;
        sfPoseGroup(MASK_ALL, 0, frameCount);
        sfWaitFrame(frameCount);
    }
}


//Проверить падение
void testDrop() {
  if( sfAbs(svImuAccX) > 50000 || sfAbs(svImuAccZ) > 50000 ) {
    //Произошло падение
      if (svImuAccX > 5000) fallingFlag = 1; // on stomach
      if (svImuAccX < 5000) fallingFlag = -1; // face up
      if (svImuAccZ > 5000) fallingFlag = 2; // on left side
      if (svImuAccZ < 5000) fallingFlag = -2; // on right side
    //Все расслабить
    //sfFreeGroup( MASK_ALL );
    //Подождать 1 сек
    //sfWaitFrame(100);
    //Перейти к вставанию
      standUp();
    }
  }

//Факторы определяют степень зависимости поворота стоп от показаний IMU
int leftRightFactor; //Фактор миксования лево-право
int frontBackFactor; //Фактор миксования вперед-назад

void mixing() {
  //Начальные установки
  leftRightFactor = 100;
  frontBackFactor = 180;
  //В бесконечном цикле выполняем подмешивание
  while(1) {
    //По боковому крену
    leftFootSideAddonMix = -svImuGyroZ * leftRightFactor >> 10;
    rightFootSideAddonMix = svImuGyroZ * leftRightFactor >> 10;
    
    //По крену вперед-назад
    rightFootFrontAddonMix = leftFootFrontAddonMix = -svImuGyroX * frontBackFactor >> 10;
    
    //Ожидаем до следующего фрейма
    sfWaitNextFrame();
    }
  }

void sitToStart() {
    frameCount = 80;
    sfPoseGroup(MASK_HEAD_TILT, 700, frameCount);
    sfPoseGroup(MASK_RIGHT_CLAVICLE, 1370, frameCount);
    sfPoseGroup(MASK_LEFT_CLAVICLE, 1370, frameCount);
    sfPoseGroup(MASK_RIGHT_ELBOW_SIDE, 700, frameCount);
    sfPoseGroup(MASK_LEFT_ELBOW_SIDE, 700, frameCount);
    sfPoseGroup(MASK_RIGHT_ELBOW, 4500, frameCount);
    sfPoseGroup(MASK_LEFT_ELBOW, 4500, frameCount);
    sfPoseGroup(MASK_RIGHT_HIP_SIDE, 410, frameCount);
    sfPoseGroup(MASK_LEFT_HIP_SIDE, -174, frameCount);
    sfPoseGroup(MASK_RIGHT_HIP, 4350, frameCount);
    sfPoseGroup(MASK_LEFT_HIP, 4240, frameCount);
    sfPoseGroup(MASK_RIGHT_KNEE, 4800, frameCount);
    sfPoseGroup(MASK_LEFT_KNEE, 5400, frameCount);
    sfPoseGroup(MASK_RIGHT_FOOT_FRONT, 2518, frameCount);
    sfPoseGroup(MASK_LEFT_FOOT_FRONT, 2550, frameCount);
    sfPoseGroup(MASK_RIGHT_FOOT_SIDE, 430, frameCount);
    sfPoseGroup(MASK_LEFT_FOOT_SIDE, -174, frameCount);
    sfPoseGroup(MASK_RIGHT_KNEE_BOT, 2050, frameCount);
    sfPoseGroup(MASK_LEFT_KNEE_BOT, 1380, frameCount);
    sfWaitFrame(frameCount);
}

void main() {
  fallingFlag = 0;
  restart_flag = 0;
  
  //Запускаем миксинг
  sfCreateTask( mixing, 20 );

  sitToStart();

  svButtonRight = SV_SLOT_INACTIVE;
  svButtonLeft = SV_SLOT_INACTIVE;
  sfBip(1, 1);
  while (svButtonPress != SV_BUTTON_RIGHT_PRESS) sfWaitFrame(1); // waithing to change parameters
  svButtonRight = SV_SLOT_RESTART_RUN;
  svButtonLeft = SV_SLOT_RELAX;
  restart_flag = 1;

  //Процесс запускаем не сразу, а через некоторое время (100 фреймов - 1 сек)
  sfWaitFrame( 100 );
  
  testDrop(); // 

  }

