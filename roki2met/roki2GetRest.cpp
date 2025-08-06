/*
  Проект "Роки-4"
  Автор
    Зубр ОВК
  Описание
    Скрипт отдых
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

  frameCount = 200;
  sfPoseGroup( MASK_ALL, 0, frameCount );
  sfWaitFrame( frameCount );
}


void sit_down(){
   frameCount = 100;
   sfPoseGroup( MASK_HIP, 3600, frameCount );
  sfPoseGroup( MASK_KNEE, 3600, frameCount );
  sfPoseGroup( MASK_FOOT_FRONT, 3600, frameCount );
   sfWaitFrame(frameCount);
}

void turn_off_knees(){
   sfFreeGroup( MASK_KNEE );
}

void main() {
  pack_hands();
  sit_down();
  turn_off_knees();
  sfWaitFrame(9000);
  pack_hands();
  
  //Вот этим мы обеспечиваем переход обратно
  sfStartSlot( sfSlotIndex("roki2Walk_Remote.cpp") );
  }
