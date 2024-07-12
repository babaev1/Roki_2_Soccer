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
  int frame4 = 100;
  int frame5 = 200;
  int frame6 = 200;
  int frame7 = 200;
  
  //1
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 606, frame1 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 521, frame1 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 176, frame1 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 378, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -65, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -232, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 3871, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW, 3898, frame1 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -161, frame1 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -164, frame1 );
  sfPoseGroup( MASK_RIGHT_HIP, 3452, frame1 );
  sfPoseGroup( MASK_LEFT_HIP, 3534, frame1 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3302, frame1 );
  sfPoseGroup( MASK_LEFT_KNEE, 3305, frame1 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3318, frame1 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3369, frame1 );
  sfPoseGroup( MASK_RIGHT_HAND, 7080, frame1 );
  sfPoseGroup( MASK_LEFT_HAND, -6947, frame1 );
  sfPoseGroup( MASK_HEAD_TILT, -23, frame1 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3506, frame1 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 3570, frame1 );
  sfPoseGroup( MASK_LEFT_CLAW, 4000, frame1 );
  sfPoseGroup( MASK_RIGHT_CLAW, -3621, frame1 );
  
  sfWaitFrame(frame1);
  
  //2
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 2164, frame2 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 2209, frame2 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -78, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, -84, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -574, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -588, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 2454, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 2465, frame2 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -126, frame2 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -88, frame2 );
  sfPoseGroup( MASK_RIGHT_HIP, 3995, frame2 );
  sfPoseGroup( MASK_LEFT_HIP, 4066, frame2 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3918, frame2 );
  sfPoseGroup( MASK_LEFT_KNEE, 3685, frame2 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3345, frame2 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3387, frame2 );
  sfPoseGroup( MASK_RIGHT_HAND, 7095, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -6928, frame2 );
  sfPoseGroup( MASK_HEAD_TILT, -22, frame2 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 2805, frame2 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 3237, frame2 );
  sfPoseGroup( MASK_LEFT_CLAW, 3984, frame2 );
  sfPoseGroup( MASK_RIGHT_CLAW, -3325, frame2 );
  
  sfWaitFrame(frame2);
  
  //4
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 1953, frame4 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 2119, frame4 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 223, frame4 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 554, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -683, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW, 1324, frame4 );
  sfPoseGroup( MASK_LEFT_HAND, -6967, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -886, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1874, frame4 );
  sfPoseGroup( MASK_RIGHT_HAND, 7095, frame4 );
  sfPoseGroup( MASK_LEFT_CLAW, 3294, frame4 );
  sfPoseGroup( MASK_RIGHT_CLAW, -2354, frame4 );
  
  sfWaitFrame(frame4);
  
  //5
  sfPoseGroup( MASK_LEFT_CLAW, 300, frame5 );
  sfPoseGroup( MASK_RIGHT_CLAW, 300, frame5 );
  
  sfWaitFrame(frame5);
  
  //6
  sfPoseGroup( MASK_RIGHT_CLAVICLE, -778, frame6 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, -697, frame6 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 393, frame6 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 592, frame6 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 66, frame6 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 599, frame6 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 5172, frame6 );
  sfPoseGroup( MASK_LEFT_ELBOW, 4725, frame6 );
  sfPoseGroup( MASK_RIGHT_HIP, 3819, frame6 );
  sfPoseGroup( MASK_LEFT_HIP, 3988, frame6 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3670, frame6 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3261, frame6 );
  sfPoseGroup( MASK_RIGHT_HAND, 7107, frame6 );
  sfPoseGroup( MASK_LEFT_HAND, -7005, frame6 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3286, frame6 );
  
  sfWaitFrame(frame6);
  
  //7
  sfPoseGroup( MASK_RIGHT_CLAVICLE, -1738, frame7 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, -1622, frame7 );
  sfPoseGroup( MASK_RIGHT_HIP, 0, frame7 );
  sfPoseGroup( MASK_LEFT_HIP, 0, frame7 );
  sfPoseGroup( MASK_RIGHT_KNEE, 0, frame7 );
  sfPoseGroup( MASK_LEFT_KNEE, 0, frame7 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frame7 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frame7 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 0, frame7 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 0, frame7 );
  
  sfWaitFrame(frame7);
  
  sfStop();

}