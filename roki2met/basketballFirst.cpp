#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)

#define MASK_SHOULDER (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)

void main() {
  int frame1 = 200;
  int frame2 = 200;
  int frame3 = 200;
  int frame4 = 300;
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 28, frame1 );
sfPoseGroup( MASK_LEFT_CLAVICLE, 4056, frame1 );
sfPoseGroup( MASK_RIGHT_SHOULDER, -28, frame1 );
sfPoseGroup( MASK_LEFT_SHOULDER, 1959, frame1 );
sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 21, frame1 );
sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -3203, frame1 );
sfPoseGroup( MASK_RIGHT_ELBOW, 25, frame1 );
sfPoseGroup( MASK_LEFT_ELBOW, 4398, frame1 );
sfPoseGroup( MASK_RIGHT_HAND, 8, frame1 );
sfPoseGroup( MASK_LEFT_HAND, -1957, frame1 );
sfPoseGroup( MASK_LEFT_CLAW, 3959, frame1 );
  
  sfWaitFrame(frame1);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 1674, frame1 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 3380, frame1 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -231, frame1 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 834, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -1558, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -3383, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 5174, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW, 3979, frame1 );
  sfPoseGroup( MASK_RIGHT_HAND, 2200, frame1 );
  sfPoseGroup( MASK_LEFT_HAND, -915, frame1 );
  sfPoseGroup( MASK_LEFT_CLAW, 3067, frame1 );
  sfPoseGroup( MASK_RIGHT_CLAW, -26, frame1 );
  
  sfWaitFrame(frame1);
  
  sfPoseGroup( MASK_LEFT_CLAW, 2370, frame1 );
  
  sfWaitFrame(frame1);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 227, frame2 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 2196, frame2 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 40, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 664, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 1737, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -61, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 540, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 3656, frame2 );
  sfPoseGroup( MASK_RIGHT_HAND, 1616, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -453, frame2 );
  sfPoseGroup( MASK_LEFT_CLAW, 2221, frame2 );
  
  sfWaitFrame(frame2);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 0, frame3 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 0, frame3 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 0, frame3 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 0, frame3 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 0, frame3 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 0, frame3 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 0, frame3 );
  sfPoseGroup( MASK_LEFT_ELBOW, 0, frame3 );
  sfPoseGroup( MASK_RIGHT_HAND, 0, frame3 );
  sfPoseGroup( MASK_LEFT_HAND, 0, frame3 );
  sfPoseGroup( MASK_LEFT_CLAW, 2221, frame3 );
  
sfWaitFrame(frame3);
  
  sfStop();
}
