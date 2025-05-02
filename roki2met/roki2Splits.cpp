/*
  Проект "Роки-4"
  Автор
    Зубр ОВК
  Описание
    Скрипт падение
*/
#include <roki2met.h>
#include <roki2global.h>

//Движения обеими ногами выполняются одновременно, поэтому делаем маски для левой и правой сторон
#define MASK_PELVIC     (MASK_LEFT_PELVIC | MASK_RIGHT_PELVIC)
#define MASK_HIP_SIDE   (MASK_LEFT_HIP_SIDE | MASK_RIGHT_HIP_SIDE)
#define MASK_HIP        (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE       (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_RIGHT_KNEE_ALL (MASK_RIGHT_KNEE | MASK_RIGHT_KNEE_BOT)
#define MASK_LEFT_KNEE_ALL       (MASK_LEFT_KNEE | MASK_LEFT_KNEE_BOT)
#define MASK_KNEE_BOT       (MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_FOOT_SIDE  (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)
#define MASK_CLAVICLE  (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_SHOULDER  (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)

//Определяет, за сколько фреймов исполнять программу на каждом шаге
int frameCount;
void pack_hands(){

  frameCount = 100;
  sfPoseGroup( MASK_ALL, 0, frameCount );
  sfPoseGroup( MASK_CLAVICLE, 6700, frameCount );
  sfPoseGroup( MASK_SHOULDER, 7400, frameCount );
  sfWaitFrame( frameCount );
}

void pack_hands_only(){
  frameCount = 50;
  sfPoseGroup( MASK_CLAVICLE, 6700, frameCount );
  sfPoseGroup( MASK_SHOULDER, 7400, frameCount );
  sfWaitFrame( frameCount );
}

void pose1(){
    frameCount = 80;
    sfPoseGroup(MASK_PELVIC, 0, frameCount);
    sfPoseGroup(MASK_HIP_SIDE, 0, frameCount);
    sfPoseGroup(MASK_HIP, 3000, frameCount);
    sfPoseGroup(MASK_KNEE, 3600, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 4100, frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfWaitFrame(frameCount);
    }

void pose2(){
    frameCount = 9;
    sfPoseGroup(MASK_FOOT_SIDE, 3000, frameCount);
    sfWaitFrame(frameCount);
    }

void pose2_2() {
    frameCount = 9;
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfWaitFrame(frameCount);
}

void pose3(){
    frameCount = 10;
    sfPoseGroup(MASK_PELVIC, 0, frameCount);
    sfPoseGroup(MASK_HIP_SIDE, 1400, frameCount);
    sfPoseGroup(MASK_HIP, 3000, frameCount);
    sfPoseGroup(MASK_KNEE, 3600, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 4100, frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 1400, frameCount);
    sfWaitFrame(frameCount);
    }

void pose4(){
    frameCount = 10;
    sfPoseGroup(MASK_HIP_SIDE, 0, frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfWaitFrame(frameCount);
    }

void pose5(){
    frameCount = 40;
    sfPoseGroup(MASK_HIP_SIDE, 4200, frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 1500, frameCount);
    sfPoseGroup( MASK_CLAVICLE, 2800, frameCount );
    sfPoseGroup( MASK_SHOULDER, 0, frameCount );
    sfWaitFrame(frameCount);
    }

void pose6() {
    frameCount = 40;
    sfPoseGroup(MASK_HIP, 0, frameCount);
    sfPoseGroup(MASK_KNEE, 0, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 0, frameCount);
    sfWaitFrame(frameCount);
}

void pose7() {
    frameCount = 80;
    sfPoseGroup(MASK_HIP, 3000, frameCount);
    sfPoseGroup(MASK_KNEE, 3600, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 4100, frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 3600, frameCount);
    sfWaitFrame(frameCount);
}

void pose8() {
    frameCount = 10;
    sfPoseGroup(MASK_HIP_SIDE, 1400, frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 1400, frameCount);
    sfWaitFrame(frameCount);
}

void pose9() {
    frameCount = 10;
    sfPoseGroup(MASK_HIP_SIDE, 0, frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfWaitFrame(frameCount);
}

void small_splits(){
  pose1();
  pose2();
  pose3();
  sfWaitFrame(100);  // time in lower position
  pose2();
  pose4();
  pack_hands();
}

void big_splits() {
    pose1();
    pose2();
    pose3();
    pose2();
    pose5();
    pose6();
    sfWaitFrame(400);
    pose7();
    pose8();
    pose2();
    pose9();
    pose2();
    pose2();
    pose2_2();
    pack_hands_only();
    pack_hands();
}

void pose10r(){
  frameCount = 50;
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 8000, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -1400, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 2500, frameCount );
  sfPoseGroup( MASK_RIGHT_KNEE_ALL, 2500, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 2500, frameCount );
  sfWaitFrame(frameCount);
}

void pose10l(){
  frameCount = 50;
  sfPoseGroup( MASK_LEFT_CLAVICLE, 8000, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -1400, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 2500, frameCount );
  sfPoseGroup( MASK_LEFT_KNEE_ALL, 2500, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 2500, frameCount );
  sfWaitFrame(frameCount);
}

void pose11r(){
   frameCount = 50;
   sfPoseGroup( MASK_RIGHT_PELVIC, 4300, frameCount );
   sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3600, frameCount );
   sfPoseGroup( MASK_RIGHT_KNEE_ALL, 1050, frameCount );
   sfWaitFrame(frameCount);
}

void pose11l(){
   frameCount = 50;
   sfPoseGroup( MASK_LEFT_PELVIC, 4300, frameCount );
   sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3600, frameCount );
   sfPoseGroup( MASK_LEFT_KNEE_ALL, 1050, frameCount );
   sfWaitFrame(frameCount);
}

void pose12r(){
   frameCount = 50;
   sfPoseGroup( MASK_RIGHT_PELVIC, 0, frameCount );
   sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
   sfPoseGroup( MASK_RIGHT_KNEE_ALL, 0, frameCount );
   sfPoseGroup( MASK_RIGHT_HIP, 0, frameCount );
   sfWaitFrame(frameCount);
}

void pose12l(){
   frameCount = 50;
   sfPoseGroup( MASK_LEFT_PELVIC, 0, frameCount );
   sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frameCount );
   sfPoseGroup( MASK_LEFT_KNEE_ALL, 0, frameCount );
   sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
   sfWaitFrame(frameCount);
}

void falling_right(){
  pose10r();
  sfWaitFrame(400);
  pose11r();
  pose12r();
  sfStartSlot( sfSlotIndex("standUpRemote.cpp") );
  
}

void falling_left(){
  pose10l();
  sfWaitFrame(400);
  pose11l();
  pose12l();
  sfStartSlot( sfSlotIndex("standUpRemote.cpp") );
  
}

void main() {
  if(splits_Mode == 2 ) big_splits();
  if (splits_Mode == 1) small_splits();
  if (splits_Mode == 3) falling_right();
  if (splits_Mode == 4) falling_left();
  splits_Mode = 0;

  //Вот этим мы обеспечиваем переход обратно
  sfStartSlot( sfSlotIndex("roki2Walk_Remote.cpp") );
  }
