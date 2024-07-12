// This is motion 'Basketball_PickUp' translated from json file 

#include <roki2met.h>

int pitStop;
int clamping;
int steps;
int frameCount;
int rotation;
float direction;


void page0(){
  frameCount = 20;
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_PELVIC, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 0, frameCount );
  sfPoseGroup( MASK_TORSO_ROTATE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 0, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, 0, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 0, frameCount );
  sfPoseGroup( MASK_HEAD_ROTATE, 0, frameCount );
  sfPoseGroup( MASK_HEAD_TILT, 0, frameCount );
  sfWaitFrame( frameCount );
    }


void page1(){
  frameCount = 200;
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -1382, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 5168, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4096, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5376, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -1382, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 5168, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -4096, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 5376, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -791 - clamping, frameCount );  //clamping
  sfPoseGroup( MASK_LEFT_SHOULDER, -791 - clamping, frameCount );
  sfWaitFrame( frameCount );
    }

void page2(){
  frameCount = 400;
  //sfPoseGroup( MASK_RIGHT_ELBOW, 921, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -791 - clamping, frameCount );  //clamping
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5376, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 700, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 700, frameCount );
  //sfPoseGroup( MASK_LEFT_ELBOW, 921, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -791 - clamping, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 5376, frameCount );
  sfWaitFrame( frameCount );
    }

void page3(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 1000, frameCount );
  sfPoseGroup( MASK_TORSO_ROTATE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 1000, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -100, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -100, frameCount );
  
  sfWaitFrame( frameCount );
    }

void page3_1(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_SHOULDER, -591 - clamping, frameCount );  //clamping
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 4096, frameCount );
  sfPoseGroup( MASK_TORSO_ROTATE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4096, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frameCount );
  sfWaitFrame( frameCount );
    }

void page4(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4608, frameCount );
  //sfPoseGroup( MASK_RIGHT_SHOULDER, -591, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 4608, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 1075, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -3072, frameCount );
  //sfPoseGroup( MASK_LEFT_SHOULDER, -591, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3840, frameCount );
  sfWaitFrame( frameCount );
    }

void page5(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_ELBOW, 1075, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -5260, frameCount );
  //sfPoseGroup( MASK_RIGHT_SHOULDER, -591, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5068, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 1382, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -2304, frameCount );
  //sfPoseGroup( MASK_LEFT_SHOULDER, -591, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3379, frameCount );
  sfWaitFrame( frameCount );
    }

void page6(){
  frameCount = 300;
  //sfPoseGroup( MASK_RIGHT_ELBOW, 1397, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1500, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4096, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -610 - clamping, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 4500, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 3225, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -821 - clamping, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -1044, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 1536, frameCount );
  sfWaitFrame( frameCount );
    }

void page7(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_ELBOW, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4096, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 3993, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 3225, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 0, frameCount );
  sfWaitFrame( frameCount );
    }

void page8(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 1689, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3072, frameCount );
  sfWaitFrame( frameCount );
    }

void page9(){
  frameCount = 100;
  sfPoseGroup( MASK_LEFT_ELBOW, 665, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4096, frameCount );
  sfWaitFrame( frameCount );
    }

void page10(){
  frameCount = 200;
  sfPoseGroup( MASK_LEFT_SHOULDER, 3840, frameCount );
  sfWaitFrame( frameCount );
    }

void page11(){
  frameCount = 200;
  sfPoseGroup( MASK_LEFT_ELBOW, 537, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, 7219, frameCount );
  sfWaitFrame( frameCount );
}

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
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, -1000, frameCount );
  
  sfWaitFrame( frameCount );
  
    frameCount = 3;
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_PELVIC, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfWaitFrame( frameCount );
  
}

void main(){
  int i;
  pitStop = 0;
  clamping = -50;
  steps = 17;
  sfQuaternionToEulerImu();
  direction = svEulerYaw;
  //while (pitStop == 0) sfWaitFrame(1); // waithing to change parameters
  page0();
  page1();
  page2();
  page3();
  for ( i = 0; i < steps; i++ ) shiftRight();
  page3_1();
  page4();
  page5();
  page6();
  page7();
  page8();
  page9();
  page10();
  page11();
  

}
