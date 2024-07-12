#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)
#define MASK_FOOT_SIDE (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)

void main() {
  int frame1 = 200;
  int frame2 = 200;
  int frame3 = 100;
  
  //1
  //Leg
  sfPoseGroup(MASK_FOOT_FRONT, -300, frame1);

  //Left Hand
  sfPoseGroup(MASK_LEFT_CLAVICLE, 4000, frame1);
  sfPoseGroup(MASK_LEFT_ELBOW_SIDE, 0, frame1);
  sfPoseGroup(MASK_LEFT_ELBOW, 0, frame1);
  
  //Right Hand
  sfPoseGroup(MASK_RIGHT_CLAVICLE, 4000, frame1);
  sfPoseGroup(MASK_RIGHT_ELBOW_SIDE, 0, frame1);
  sfPoseGroup(MASK_RIGHT_ELBOW, 0, frame1);
  
  sfWaitFrame(frame1);
  
  //2
  sfPoseGroup(MASK_FOOT_FRONT, -300, frame1);
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 4852, frame2 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4061, frame2 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 100, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 3131, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4218, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -4062, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 2751, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 4983, frame2 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -122, frame2 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -225, frame2 );
  sfPoseGroup( MASK_RIGHT_HAND, -472, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -3947, frame2 );
  sfPoseGroup( MASK_RIGHT_CLAW, -2279, frame2 );
  
  sfWaitFrame(frame2);
  
  //3  
  sfStop();

}