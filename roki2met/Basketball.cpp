#include <roki2met.h>

#define MASK_ELBOW_SIDE (MASK_LEFT_ELBOW_SIDE | MASK_RIGHT_ELBOW_SIDE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_SHOULDER (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)


void main() {
  int frame1 = 100;
  int frame2 = 200;
  int frame3 = 100;
  int frame4 = 100;
  int frame5 = 300;
  int frame6 = 200;
  int frame7 = 100;
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, -373, frame1 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, -567, frame1 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -23, frame1 );
  sfPoseGroup( MASK_LEFT_SHOULDER, -28, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -22, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -23, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 3232, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW, 3602, frame1 );
  sfPoseGroup( MASK_RIGHT_PELVIC, 32, frame1 );
  sfPoseGroup( MASK_LEFT_PELVIC, 24, frame1 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -90, frame1 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -85, frame1 );
  sfPoseGroup( MASK_RIGHT_HIP, 3044, frame1 );
  sfPoseGroup( MASK_LEFT_HIP, 3079, frame1 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3331, frame1 );
  sfPoseGroup( MASK_LEFT_KNEE, 2933, frame1 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3302, frame1 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3369, frame1 );
  sfPoseGroup( MASK_RIGHT_HAND, -13, frame1 );
  sfPoseGroup( MASK_LEFT_HAND, 25, frame1 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3310, frame1 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 3810, frame1 );
  sfPoseGroup( MASK_LEFT_CLAW, 1, frame1 );
  sfPoseGroup( MASK_RIGHT_CLAW, 8, frame1 );
  
  sfWaitFrame(frame1);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, -396, frame1 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, -542, frame1 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 325, frame1 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 374, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -45, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -44, frame1 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 3202, frame1 );
  sfPoseGroup( MASK_LEFT_ELBOW, 3567, frame1 );
  sfPoseGroup( MASK_RIGHT_PELVIC, 60, frame1 );
  sfPoseGroup( MASK_LEFT_PELVIC, -9, frame1 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -115, frame1 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -110, frame1 );
  sfPoseGroup( MASK_RIGHT_HIP, 3736, frame1 );
  sfPoseGroup( MASK_LEFT_HIP, 3495, frame1 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3370, frame1 );
  sfPoseGroup( MASK_LEFT_KNEE, 3008, frame1 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3257, frame1 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3397, frame1 );
  sfPoseGroup( MASK_RIGHT_HAND, -12, frame1 );
  sfPoseGroup( MASK_LEFT_HAND, 23, frame1 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3395, frame1 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 3861, frame1 );
  sfPoseGroup( MASK_LEFT_CLAW, 1, frame1 );
  sfPoseGroup( MASK_RIGHT_CLAW, 12, frame1 );
  
  sfWaitFrame(frame1);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 2634, frame2 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 2394, frame2 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -106, frame2 );
  sfPoseGroup( MASK_LEFT_SHOULDER, -114, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -2068, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -1839, frame2 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1161, frame2 );
  sfPoseGroup( MASK_LEFT_ELBOW, 1636, frame2 );
  sfPoseGroup( MASK_RIGHT_PELVIC, 88, frame2 );
  sfPoseGroup( MASK_LEFT_PELVIC, 14, frame2 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -90, frame2 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -78, frame2 );
  sfPoseGroup( MASK_RIGHT_HIP, 4778, frame2 );
  sfPoseGroup( MASK_LEFT_HIP, 4852, frame2 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3340, frame2 );
  sfPoseGroup( MASK_LEFT_KNEE, 3017, frame2 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3310, frame2 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3402, frame2 );
  sfPoseGroup( MASK_RIGHT_HAND, 1484, frame2 );
  sfPoseGroup( MASK_LEFT_HAND, -4318, frame2 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3360, frame2 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 3876, frame2 );
  sfPoseGroup( MASK_LEFT_CLAW, 3008, frame2 );
  sfPoseGroup( MASK_RIGHT_CLAW, -23, frame2 );
  
  sfWaitFrame(frame2);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 3182, frame3 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 2762, frame3 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -217, frame3 );
  sfPoseGroup( MASK_LEFT_SHOULDER, -247, frame3 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -4244, frame3 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -4443, frame3 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1224, frame3 );
  sfPoseGroup( MASK_LEFT_ELBOW, 1289, frame3 );
  sfPoseGroup( MASK_RIGHT_PELVIC, 124, frame3 );
  sfPoseGroup( MASK_LEFT_PELVIC, -43, frame3 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -72, frame3 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -31, frame3 );
  sfPoseGroup( MASK_RIGHT_HIP, 4912, frame3 );
  sfPoseGroup( MASK_LEFT_HIP, 5023, frame3 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3340, frame3 );
  sfPoseGroup( MASK_LEFT_KNEE, 3012, frame3 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3365, frame3 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3395, frame3 );
  sfPoseGroup( MASK_RIGHT_HAND, 438, frame3 );
  sfPoseGroup( MASK_LEFT_HAND, 25, frame3 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3363, frame3 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 3888, frame3 );
  sfPoseGroup( MASK_LEFT_CLAW, 2665, frame3 );
  sfPoseGroup( MASK_RIGHT_CLAW, 116, frame3 );
  
  sfWaitFrame(frame3);
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 2908, frame4 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 2908, frame4 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -266, frame4 );
  sfPoseGroup( MASK_LEFT_SHOULDER, -258, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, -3746, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -4002, frame4 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 1598, frame4 );
  sfPoseGroup( MASK_LEFT_ELBOW, 1294, frame4 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -56, frame4 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -33, frame4 );
  sfPoseGroup( MASK_RIGHT_HIP, 4966, frame4 );
  sfPoseGroup( MASK_LEFT_HIP, 5068, frame4 );
  sfPoseGroup( MASK_RIGHT_KNEE, 3327, frame4 );
  sfPoseGroup( MASK_LEFT_KNEE, 3034, frame4 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3369, frame4 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3395, frame4 );
  sfPoseGroup( MASK_RIGHT_HAND, 462, frame4 );
  sfPoseGroup( MASK_LEFT_HAND, 152, frame4 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3381, frame4 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 3875, frame4 );
  sfPoseGroup( MASK_LEFT_CLAW, 2131, frame4 );
  sfPoseGroup( MASK_RIGHT_CLAW, 117, frame4 );
  sfPoseGroup( MASK_LEFT_PELVIC, -350, frame4 );
  sfPoseGroup( MASK_RIGHT_PELVIC, -350, frame4 );
  
  sfWaitFrame(frame4);
  
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -77, frame5 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, -12, frame5 );
  sfPoseGroup( MASK_RIGHT_HIP, 2876, frame5 );
  sfPoseGroup( MASK_LEFT_HIP, 3064, frame5 );
  sfPoseGroup( MASK_RIGHT_KNEE, 229, frame5 );
  sfPoseGroup( MASK_LEFT_KNEE, 371, frame5 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 3351, frame5 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 3363, frame5 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 6025, frame5 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, 6136, frame5 );
  sfPoseGroup( MASK_LEFT_CLAW, 2155, frame5 );
  sfPoseGroup( MASK_RIGHT_CLAW, 120, frame5 );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, -114, frame5 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, -486, frame5 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 107, frame5 );
  sfPoseGroup( MASK_LEFT_SHOULDER, 146, frame5 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 35, frame5 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -561, frame5 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 345, frame5 );
  sfPoseGroup( MASK_LEFT_ELBOW, 742, frame5 );
  sfPoseGroup( MASK_RIGHT_HAND, -301, frame5 );
  sfPoseGroup( MASK_LEFT_HAND, 421, frame5 );
  sfPoseGroup( MASK_LEFT_CLAW, 2131, frame5 );
  sfPoseGroup( MASK_RIGHT_CLAW, 124, frame5 );
  sfPoseGroup( MASK_LEFT_PELVIC, -350, frame4 );
  sfPoseGroup( MASK_RIGHT_PELVIC, -350, frame4 );
  
  sfWaitFrame(frame5);
  sfPoseGroup( MASK_LEFT_CLAVICLE, -458, frame6 );
sfPoseGroup( MASK_LEFT_SHOULDER, 118, frame6 );
sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -580, frame6 );
sfPoseGroup( MASK_LEFT_ELBOW, 769, frame6 );
sfPoseGroup( MASK_RIGHT_HIP, 3541, frame6 );
sfPoseGroup( MASK_LEFT_HIP, 3635, frame6 );
sfPoseGroup( MASK_RIGHT_KNEE, 3632, frame6 );
sfPoseGroup( MASK_LEFT_KNEE, 3554, frame6 );
sfPoseGroup( MASK_LEFT_HAND, 406, frame6 );
sfPoseGroup( MASK_RIGHT_KNEE_BOT, 3369, frame6 );
sfPoseGroup( MASK_LEFT_KNEE_BOT, 3491, frame6 );
sfPoseGroup( MASK_LEFT_CLAW, 2138, frame6 );
  sfWaitFrame(frame6);
  
  
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 31, frame6 );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 27, frame6 );
  sfPoseGroup( MASK_RIGHT_SHOULDER, -25, frame6 );
  sfPoseGroup( MASK_LEFT_SHOULDER, -4, frame6 );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 12, frame6 );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, -15, frame6 );
  sfPoseGroup( MASK_RIGHT_ELBOW, 25, frame6 );
  sfPoseGroup( MASK_LEFT_ELBOW, 26, frame6 );
  sfPoseGroup( MASK_RIGHT_PELVIC, 6, frame6 );
  sfPoseGroup( MASK_LEFT_PELVIC, -12, frame6 );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -23, frame6 );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, 21, frame6 );
  sfPoseGroup( MASK_RIGHT_HIP, 25, frame6 );
  sfPoseGroup( MASK_LEFT_HIP, 28, frame6 );
  sfPoseGroup( MASK_RIGHT_KNEE, -21, frame6 );
  sfPoseGroup( MASK_LEFT_KNEE, -24, frame6 );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 24, frame6 );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 25, frame6 );
  sfPoseGroup( MASK_RIGHT_HAND, -15, frame6 );
  sfPoseGroup( MASK_LEFT_HAND, 7, frame6 );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, -23, frame6 );
  sfPoseGroup( MASK_LEFT_KNEE_BOT, -24, frame6 );
  sfPoseGroup( MASK_LEFT_CLAW, 2131, frame6 );
  sfPoseGroup( MASK_RIGHT_CLAW, 13, frame6 );
  
  sfWaitFrame(frame6);

   
  sfStop();
}
