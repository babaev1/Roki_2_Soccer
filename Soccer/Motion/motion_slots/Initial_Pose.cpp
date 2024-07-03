// This is motion 'Initial_Pose' translated from json file 

#include <roki2met.h>
int frameCount;

void main(){

// page 0
  frameCount = 20;
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_KNEE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_PELVIC, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_SHOULDER, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_CLAVICLE, 0, frameCount );
  sfPoseGroup( MASK_TORSO_ROTATE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_FRONT, 0, frameCount );
  sfPoseGroup( MASK_LEFT_KNEE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP, 0, frameCount );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_PELVIC, 0, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW, 0, frameCount );
  sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_SHOULDER, 0, frameCount );
  sfPoseGroup( MASK_LEFT_CLAVICLE, 0, frameCount );
  sfWaitFrame( frameCount );
}

