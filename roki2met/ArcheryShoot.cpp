#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)
#define MASK_FOOT_SIDE (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)

void main() {
  int frame1 = 100;
  int frame2 = 100;
  int frame3 = 100;
  int frame4 = 100;
  
  //1
  sfPoseGroup( MASK_HEAD_ROTATE, 0, frame2);
  sfPoseGroup( MASK_FOOT_FRONT, -200, frame2);
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5052, frame2 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4061, frame2 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 0, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 3131, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4218, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -3762, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 2751, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 4983, frame2 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -122, frame2 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -225, frame2 );
  sfPoseGroup( MASK_RIGHT_HAND, -672, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -3947, frame2 );
  sfPoseGroup( MASK_RIGHT_CLAW, -2279, frame2 );
  
  
  sfWaitFrame(frame2);
  
  //3
  sfPoseGroup( MASK_HEAD_ROTATE, 0, frame2);
  sfPoseGroup( MASK_FOOT_FRONT, -200, frame2);
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4061, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 3131, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -3362, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 5083, frame2 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -122, frame2 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -225, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -3947, frame2 );
  
  sfPoseGroup( MASK_RIGHT_CLAW, 230, frame2 ); 
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5338, frame2 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 146, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4124, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 2987, frame2 );
  sfPoseGroup( MASK_RIGHT_HAND, -829, frame2 );
  
  
  sfWaitFrame(frame2);
  
  //3
  sfPoseGroup( MASK_HEAD_ROTATE, 3500, frame4);
  sfPoseGroup( MASK_FOOT_FRONT, 1000, frame4);
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 4431, frame4 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3851, frame4 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 4053, frame4 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 2311, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -3426, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -3070, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 5317, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW, 4704, frame4 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -172, frame4 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -320, frame4 );
  sfPoseGroup( MASK_RIGHT_HAND, -250, frame4 );
  sfPoseGroup( MASK_LEFT_HAND, -4621, frame4 );
  sfPoseGroup( MASK_RIGHT_CLAW, 230, frame4 );
  
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -267, frame4 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -250, frame4 );
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 5716, frame4 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3732, frame4 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 4746, frame4 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 1601, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4653, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -2762, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 4687, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW, 4068, frame4 );
  sfPoseGroup( MASK_RIGHT_HAND, 496, frame4 );
  sfPoseGroup( MASK_LEFT_HAND, -5171, frame4 );
  sfPoseGroup( MASK_RIGHT_CLAW, 127, frame4 );
  
  sfStop();

}