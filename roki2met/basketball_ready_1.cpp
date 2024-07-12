// Initial pose for Basketball FIRA

#include <roki2met.h>
int frameCount;

void main(){
  frameCount = 80;
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, 525, frameCount );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, 544, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 589, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 538, frameCount );
  sfWaitFrame( frameCount );
  
    frameCount = 150;
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -781, frameCount );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, 1605, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 1954, frameCount );
  sfPoseGroup( MASK_RIGHT_KNEE, 3760, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 1658, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, -1072, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 1020, frameCount );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, -68, frameCount );
  
  
  sfWaitFrame( frameCount );
  
  frameCount = 50;
  sfPoseGroup( MASK_RIGHT_HIP_SIDE, -730, frameCount );
  sfPoseGroup( MASK_LEFT_HIP_SIDE, 730, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, -670, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 670, frameCount );
  sfPoseGroup( MASK_RIGHT_HIP, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_KNEE, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_KNEE_BOT, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 0, frameCount );
   
  sfWaitFrame( frameCount );

}