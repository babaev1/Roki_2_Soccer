// Initial pose for Basketball FIRA

#include <roki2met.h>

int pitStop;
int steps;
int frameCount;
int rotation;
float direction;


void shiftRight(){
  sfQuaternionToEulerImu();
  rotation = (direction - svEulerYaw) * ENC_PER_RADIAN;
  //rotation = 0;
  
  frameCount = 3;
  //sfPoseGroup( MASK_RIGHT_PELVIC, rotation, frameCount );
  sfWaitFrame( frameCount );
  
  frameCount = 3;
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 1103, frameCount );
  sfPoseGroup( MASK_RIGHT_PELVIC, rotation, frameCount );
  //sfPoseGroup( MASK_LEFT_PELVIC, -200, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, -1000, frameCount );
  
  sfWaitFrame( frameCount );
  
    frameCount = 3;
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_PELVIC, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfWaitFrame( frameCount );
  
  frameCount = 3;
  //sfPoseGroup( MASK_RIGHT_PELVIC, 0, frameCount );
  sfWaitFrame( frameCount );

}

void main(){
  int i;
  pitStop = 0;
  steps = 17;
  sfQuaternionToEulerImu();
  direction = svEulerYaw;
  //while (pitStop == 0) sfWaitFrame(1); // waithing to change parameters
  for ( i = 0; i < steps; i++ ) shiftRight();
}