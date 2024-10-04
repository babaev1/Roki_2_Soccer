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
  sfPoseGroup( MASK_RIGHT_SHOULDER, -300 - clamping, frameCount );  //clamping
  sfPoseGroup( MASK_LEFT_SHOULDER, -300 - clamping, frameCount );
  sfWaitFrame( frameCount );
    }

void page2(){
  frameCount = 400;
  sfPoseGroup( MASK_RIGHT_SHOULDER, -791 - clamping, frameCount );  //clamping
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5376, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 700, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 700, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -791 - clamping, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 5376, frameCount );
  sfWaitFrame( frameCount );
    }

void page3(){
  frameCount = 300;
  sfPoseGroup( MASK_RIGHT_HIP, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -100, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -100, frameCount );
  sfWaitFrame( frameCount );
    }

void page3_0(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 1000, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 1000, frameCount );
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
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 742, frameCount );
  
  sfWaitFrame( frameCount );
    }

void page4(){
  frameCount = 100;
  sfPoseGroup( MASK_LEFT_ELBOW, 1075, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -3072, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3840, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 4544, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -290, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4631, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1220, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1321, frameCount );
  
  sfWaitFrame( frameCount );
    }

void page5(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_ELBOW, 1075, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -5260, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5068, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 1382, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -2304, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3379, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1450, frameCount );
sfPoseGroup( MASK_RIGHT_ELBOW, 1638, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 1430, frameCount );
  
  sfWaitFrame( frameCount );
    }

void page6(){
  frameCount = 300;
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5782, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 2713, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -799, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -845, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -6354, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -1477, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1609, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 1947, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5726, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -1221, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -1116, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -6638, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -1177, frameCount );
  
  sfWaitFrame( frameCount );
    }

void page7(){
  frameCount = 300;
  //sfPoseGroup( MASK_RIGHT_ELBOW, 1397, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 3225, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -821 - clamping, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -1044, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 1536, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5628, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -1516, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -6475, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1417, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, -1145, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -6106, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1816, frameCount );
  
  sfWaitFrame( frameCount );
    }

void page8(){
  frameCount = 300;
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 6268, frameCount );
  sfWaitFrame( frameCount );
    }

void page9(){
  frameCount = 100;
  sfPoseGroup( MASK_RIGHT_SHOULDER, 1009, frameCount );
  sfWaitFrame( frameCount );
    }

void page10(){
  frameCount = 200;
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 1689, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3072, frameCount );
  sfWaitFrame( frameCount );
    }

void page11(){
  frameCount = 100;
  sfPoseGroup( MASK_LEFT_ELBOW, 665, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4096, frameCount );
  sfWaitFrame( frameCount );
    }

void page12(){
  frameCount = 200;
  sfPoseGroup( MASK_LEFT_SHOULDER, 3840, frameCount );
  sfWaitFrame( frameCount );
    }

void page13(){
  frameCount = 200;
  sfPoseGroup( MASK_LEFT_ELBOW, 537, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, 7219, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_HEAD_ROTATE, -420, frameCount );
  sfPoseGroup( MASK_TORSO_ROTATE, 420, frameCount );
  sfWaitFrame( frameCount );
}

void shiftRight(){
  sfQuaternionToEulerImu();
  rotation = (direction - svEulerYaw) * ENC_PER_RADIAN * 1.2;
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
  sfWaitFrame( frameCount * 8 );
  
}

void main(){
  int i;
  frameCount = 1;
  pitStop = 0;
  clamping = -50;
  steps = 17;
  sfQuaternionToEulerImu();
  direction = svEulerYaw;
  while (pitStop == 0) sfWaitFrame(1); // waithing to change parameters
  page0();
  page1();
  page2();
  page3();
  page3_0();
  //for ( i = 0; i < steps; i++ ) shiftRight();
  //page3_1();
  //page4();
  //page5();
  //page6();
  //page7();
  //page8();
  //page9();
  //page10();
  //page11();
  //page12();
  //page13();
 
}
