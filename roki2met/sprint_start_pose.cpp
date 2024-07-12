// Initial pose for Sprint FIRA




#include <roki2met.h>
int frameCount;

void main(){
  frameCount = 80;
sfPoseGroup( MASK_HEAD_TILT, 700, frameCount );
sfPoseGroup( MASK_RIGHT_CLAVICLE, 1370, frameCount );
sfPoseGroup( MASK_LEFT_CLAVICLE, 1370, frameCount );
sfPoseGroup( MASK_RIGHT_ELBOW_SIDE, 700, frameCount );
sfPoseGroup( MASK_LEFT_ELBOW_SIDE, 700, frameCount );
sfPoseGroup( MASK_RIGHT_ELBOW, 4500, frameCount );
sfPoseGroup( MASK_LEFT_ELBOW, 4500, frameCount );
sfPoseGroup( MASK_RIGHT_HIP_SIDE, 410, frameCount );
sfPoseGroup( MASK_LEFT_HIP_SIDE, -174, frameCount );
sfPoseGroup( MASK_RIGHT_HIP, 4350, frameCount );
sfPoseGroup( MASK_LEFT_HIP, 4240, frameCount );
sfPoseGroup( MASK_RIGHT_KNEE, 4800, frameCount );
sfPoseGroup( MASK_LEFT_KNEE, 5400, frameCount );
sfPoseGroup( MASK_RIGHT_FOOT_FRONT, 2518, frameCount );
sfPoseGroup( MASK_LEFT_FOOT_FRONT, 2550, frameCount );
sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 430, frameCount );
sfPoseGroup( MASK_LEFT_FOOT_SIDE, -174, frameCount );
sfPoseGroup( MASK_RIGHT_KNEE_BOT, 2050, frameCount );
sfPoseGroup( MASK_LEFT_KNEE_BOT, 1380, frameCount );
sfWaitFrame( frameCount );
sfFreeGroup( MASK_RIGHT_KNEE| MASK_LEFT_KNEE );

}