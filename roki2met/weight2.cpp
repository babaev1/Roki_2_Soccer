#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)
#define MASK_FOOT_SIDE (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)

void main() {
  int frame1 = 300;
  int frame2 = 300;
  int frame3 = 300;
  int frame4 = 100;
  int frame5 = 200;
  int frame6 = 200;
  int frame7 = 200;
  
  //1
  sfPoseGroup( MASK_RIGHT_CLAVICLE, -1738, frame1 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, -1622, frame1 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 393, frame1 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 592, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 66, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 599, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 5172, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW, 4725, frame1 );
  sfPoseGroup( MASK_RIGHT_HAND, 7107, frame1 );
  sfPoseGroup( MASK_LEFT_HAND, -7005, frame1 );
  sfPoseGroup( MASK_LEFT_CLAW, 350, frame1 );
  sfPoseGroup( MASK_RIGHT_CLAW, 224, frame1 );
  
  sfWaitFrame(frame1);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 4953, frame2 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 4587, frame2 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 200, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 310, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -417, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 287, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 4071, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 3987, frame2 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, -131, frame2 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, -146, frame2 );
  sfPoseGroup( MASK_RIGHT_HAND, 7208, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -7005, frame2 );
  
  sfWaitFrame(frame2);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 6357, frame3 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 6128, frame3 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frame3 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frame3 );
  
  sfWaitFrame(frame3);
  
  
  sfStop();

}