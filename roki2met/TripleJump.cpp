#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)
#define MASK_FOOT_SIDE (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)

void main() {
  
  int frame3 = 25; //25
  int frame4 = 25; //
  int frame5 = 25;
  int frame6 = 25;  
  int frame7 = 50;
  
  // Up
  sfPoseGroup( MASK_CLAVICLE, -800, 200 );
  sfPoseGroup( MASK_ELBOW, 3000, 200);
  //Under
  sfPoseGroup( MASK_FOOT_FRONT, 3000, 200 );
  sfPoseGroup( MASK_HIP, 3300, 200 );
  sfPoseGroup( MASK_KNEE, 3100, 200 );
  sfPoseGroup( MASK_FOOT_SIDE, 0, 200);

  sfWaitFrame(200);
  
  // 2
  // Up
  sfPoseGroup( MASK_CLAVICLE, 2000, 200 );
  sfPoseGroup( MASK_ELBOW, 1000, 200);
  //Under
  sfPoseGroup( MASK_FOOT_FRONT, 3200, 200 );
  sfPoseGroup( MASK_HIP, 3500, 200 );
  sfPoseGroup( MASK_KNEE, 3100, 200 );
  sfPoseGroup( MASK_FOOT_SIDE, 0, 200);

  sfWaitFrame(200);
  
  // 3
  // Up
  sfPoseGroup( MASK_CLAVICLE, 7500, frame3 );
  sfPoseGroup( MASK_ELBOW, 3000, frame3 );
  //Under
  sfPoseGroup( MASK_FOOT_FRONT, 1000, frame3 );
  sfPoseGroup( MASK_HIP, 500, frame3 );
  sfPoseGroup( MASK_KNEE, 0, frame3 );
  sfPoseGroup( MASK_FOOT_SIDE, 1000, frame3 );
  
  sfWaitFrame(frame3);

  // 4
  // Up
  sfPoseGroup( MASK_CLAVICLE, -800, frame4 );
  sfPoseGroup( MASK_ELBOW, 4000, frame4 );
  //Under
  sfPoseGroup( MASK_FOOT_FRONT, 2800, frame4 );
  sfPoseGroup( MASK_HIP, 3200, frame4 );
  sfPoseGroup( MASK_KNEE, 3200, frame4 );
  sfPoseGroup( MASK_FOOT_SIDE, 0, frame4 );

  sfWaitFrame(frame4);
  
  // 5
  // Up
  sfPoseGroup( MASK_CLAVICLE, -800, frame5 );
  sfPoseGroup( MASK_ELBOW, 4000, frame5 );
  //Under
  sfPoseGroup( MASK_FOOT_FRONT, 2800, frame5 );
  sfPoseGroup( MASK_HIP, 4200, frame5 );
  sfPoseGroup( MASK_KNEE, 3100, frame5 );
  sfPoseGroup( MASK_FOOT_SIDE, 0, frame5 );
  
  sfWaitFrame(frame5);
  
//  // 6
//  // Up
//  sfPoseGroup( MASK_CLAVICLE, -800, frame6 );
//  sfPoseGroup( MASK_ELBOW, 4000, frame6 );
//  //Under
//  sfPoseGroup( MASK_FOOT_FRONT, 3000, frame6 );
//  sfPoseGroup( MASK_HIP, 2400, frame6 );
//  sfPoseGroup( MASK_KNEE, 3100, frame6 );
//  sfPoseGroup( MASK_FOOT_SIDE, 0, frame6 );
  
//  sfWaitFrame(frame6);  
  
//  // 7
//  // Up
//  sfPoseGroup( MASK_CLAVICLE, -800, frame7 );
//  sfPoseGroup( MASK_ELBOW, 4000, frame7 );
//  //Under
//  sfPoseGroup( MASK_FOOT_FRONT, 3300, frame7 );
//  sfPoseGroup( MASK_HIP, 2400, frame7 );
//  sfPoseGroup( MASK_KNEE, 3100, frame7 );
//  sfPoseGroup( MASK_FOOT_SIDE, 0, frame7 );

  sfStop();
}