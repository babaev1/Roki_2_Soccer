#include <roki2met.h>

#define MASK_ELBOW_SIDE (MASK_LEFT_ELBOW_SIDE | MASK_RIGHT_ELBOW_SIDE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_SHOULDER (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)


void main() {
  int frame1 = 100;
  int frame2 = 20;
//  int frame3 = 100;
//  int frame4 = 100;
//  int frame5 = 300;
//  int frame6 = 100;
//  int frame7 = 100;
  
  sfPoseGroup( MASK_LEFT_CLAVICLE, -3162, frame1 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 331, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 8259, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW, 525, frame1 );
  sfPoseGroup( MASK_LEFT_HAND, -398, frame1 );

  sfWaitFrame(frame1);
  
  //2
  sfPoseGroup( MASK_LEFT_CLAVICLE, -5445, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 324, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 8175, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 2634, frame2 );
  
  sfWaitFrame(frame2);
  
  sfStop();
}
