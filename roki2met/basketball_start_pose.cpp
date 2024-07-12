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

}