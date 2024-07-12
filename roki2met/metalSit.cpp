#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)

int angle;
int velo;
int factor;
int frames;


void main() {
  //Начальные значения
  velo = 20;
  angle = 2000;
  factor = 100;

  //sfPoseGroup( MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT, 2000, 100 );
  
  //Вытянуться в 0
  sfPoseGroup( MASK_ALL, 0, 100 );
  //svPoseGroup( MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER, 300, 100 );
  sfWaitFrame( 100 );
  
  //Подготавливаем и запускаем аккумулятор
  sfAccumSetup4( &leftFootFrontPwm, &leftKneeCurrent, &leftFootFrontTarget, &leftKneeTarget );
  sfAccumTrigger();
  
  while(1) {
    frames = angle / velo;
    
    //Присели
    //svPoseGroupDelay( MASK_HIP | MASK_FOOT_FRONT, angle + 25, frames, 0 );
    sfPoseGroupDelay( MASK_HIP | MASK_FOOT_FRONT, angle, frames, 0 );
    sfPoseGroup( MASK_KNEE, angle, frames * factor / 100 );
    sfWaitFrame( frames );
    sfWaitFrame( 200 );
    
    //Встали
//    svPoseGroupDelay( MASK_HIP | MASK_FOOT_FRONT, 0, frames, 2 );
    sfPoseGroup( MASK_HIP | MASK_FOOT_FRONT, 0, frames );
    sfPoseGroup( MASK_KNEE, 0, frames );
    sfWaitFrame( frames );

    sfWaitFrame( 200 );
    }
  }
