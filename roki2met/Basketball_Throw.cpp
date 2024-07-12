// This is motion 'Basketball_PickUp' translated from json file 

#include <roki2met.h>

int pitStop;
int startStop;
int distance;
int direction;
int frameCount;

void page13(){
  frameCount = 200;
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4096, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 537, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, 7219, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_HEAD_ROTATE, -188, frameCount );
  sfPoseGroup( MASK_TORSO_ROTATE, 420, frameCount );
  sfWaitFrame( frameCount );
}

void page14(){
  frameCount = 400;
  sfPoseGroup( MASK_LEFT_ELBOW, -1000 + distance, frameCount ); //-550
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, - direction, frameCount );
  sfWaitFrame( frameCount );
}

void page15(){
  frameCount = 8;
  sfPoseGroup( MASK_LEFT_ELBOW, 1730 + distance, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 1365, frameCount );
  sfWaitFrame( frameCount );
}

void page16(){
  frameCount = 12;
  sfPoseGroup( MASK_LEFT_ELBOW, 3000, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 0, frameCount );
  sfWaitFrame( frameCount );
}

void page17(){
  frameCount = 100;
  sfPoseGroup( MASK_ALL, 0, frameCount );
  sfWaitFrame( frameCount );
}


void main(){
  frameCount = 1;
  pitStop = 0;
  startStop = 0;
  distance = 3000;
  direction = 200;
  while (pitStop == 0) sfWaitFrame(1); // waiting to change parameters
  page13();
  while (startStop == 0) sfWaitFrame(1); // waiting to change parameters
  page14();
  //sfWaitFrame( 100 );
  page15();
  //page16();
  page17();
  
 
}
