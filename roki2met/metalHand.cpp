#include <roki2met.h>


#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)

#define MASK_SHOULDER (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)

int angle;
int velo;
int factor;
int frames;

void move( int ang, int fr ) {
  sfPoseGroup( MASK_SHOULDER, ang, fr );
  sfPoseGroup( MASK_ELBOW, ang * 2, fr );
  sfWaitFrame( fr );
  }

void main() {
  //Начальные значения
  velo = 20;
  angle = 2000;
  factor = 100;

//  sfPoseGroup( MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE, -8000, 100 );
  //sfPoseGroup( MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW, 6000, 100 );
  
  //Вытянуться в 0
  sfPoseGroup( MASK_ALL, 0, 100 );
  //svPoseGroup( MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER, 300, 100 );
  sfWaitFrame( 100 );
  
  //Подготавливаем и запускаем аккумулятор
  sfAccumSetup4( &leftFootFrontCurrent, &leftKneeCurrent, &leftFootFrontTarget, &leftKneeTarget );
  sfAccumTrigger();
  
  while(1) {
    frames = angle / velo;
    
    //Развели
    move( angle, frames );

    sfWaitFrame( 50 );
    
    //Свели
    move( 0, frames );
    sfWaitFrame( 50 );
    }
  }

