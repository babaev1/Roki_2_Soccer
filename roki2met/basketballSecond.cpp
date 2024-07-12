#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)

#define MASK_SHOULDER (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)

void main() {
  int frame1 = 200;
  int frame2 = 200;
  int frame3 = 10;
  int frame4 = 300;

  sfPoseGroup( MASK_LEFT_CLAVICLE, -408, frame1 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 3226, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 2204, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW, 1753, frame1 );
  sfPoseGroup( MASK_LEFT_HAND, -1452, frame1 );
  sfPoseGroup( MASK_LEFT_CLAW, 2192, frame1 );
  
  sfWaitFrame(frame1);
  
  sfPoseGroup( MASK_LEFT_CLAVICLE, 6331, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, -383, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -2906, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 996, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -3862, frame2 );
  sfPoseGroup( MASK_LEFT_CLAW, 2210, frame2 );
  
  sfWaitFrame(frame2);
  sfPoseGroup( MASK_LEFT_CLAW, 4000, frame3 );
  sfStop();
}
