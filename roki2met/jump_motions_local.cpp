// This is jump motions for presize movements. 

#include <roki2met.h>
//#include <roki2global.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT )
//#define MASK_KNEE_BOT ( MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_FOOT_SIDE (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)
#define MASK_SHOULDER (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)

int pitStop;
int frameCount;
int factor;
int jump_mode_local;

void jump_turn_CCW() {
	frameCount = 20;
	sfWaitFrame(frameCount);
	frameCount = 6;
	sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
	sfPoseGroup(MASK_RIGHT_PELVIC, 160 * factor, frameCount);
	sfPoseGroup(MASK_LEFT_PELVIC, -160 * factor, frameCount);
	sfWaitFrame(frameCount);
	sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
	sfPoseGroup(MASK_RIGHT_PELVIC, 0, frameCount);
	sfPoseGroup(MASK_LEFT_PELVIC, 0, frameCount);
	sfWaitFrame(frameCount);
}

void jump_turn_CW() {
	frameCount = 20;
	sfWaitFrame(frameCount);
	frameCount = 6;
	sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
	sfPoseGroup(MASK_RIGHT_PELVIC, -160 * factor, frameCount);
	sfPoseGroup(MASK_LEFT_PELVIC, 160 * factor, frameCount);
	sfWaitFrame(frameCount);
	sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
	sfPoseGroup(MASK_RIGHT_PELVIC, 0, frameCount);
	sfPoseGroup(MASK_LEFT_PELVIC, 0, frameCount);
	sfWaitFrame(frameCount);
}

void jump_forward() {
	frameCount = 30;
  	sfPoseGroup(MASK_FOOT_FRONT, 16 * factor, frameCount);
	sfWaitFrame(frameCount);
	frameCount = 9;
	sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
  	sfPoseGroup(MASK_FOOT_FRONT, -48 * factor, frameCount);
	sfWaitFrame(frameCount);
	sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
	sfPoseGroup(MASK_FOOT_FRONT, 16 * factor, frameCount);
	sfWaitFrame(frameCount);
  	sfPoseGroup(MASK_FOOT_FRONT, 0, frameCount);
	sfWaitFrame(frameCount);
}

void jump_backward() {
	frameCount = 30;
  	sfPoseGroup(MASK_FOOT_FRONT, -16 * factor, frameCount);
	sfWaitFrame(frameCount);
	frameCount = 9;
	sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
  	sfPoseGroup(MASK_FOOT_FRONT, 48 * factor, frameCount);
	sfWaitFrame(frameCount);
	sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
	sfPoseGroup(MASK_FOOT_FRONT, -16 * factor, frameCount);
	sfWaitFrame(frameCount);
  	sfPoseGroup(MASK_FOOT_FRONT, 0, frameCount);
	sfWaitFrame(frameCount);
}

void jump_left() {
	frameCount = 30;
  	sfPoseGroup(MASK_LEFT_FOOT_SIDE, -30 * factor, frameCount);
  	sfPoseGroup(MASK_RIGHT_FOOT_SIDE, 30 * factor, frameCount);
  	sfPoseGroup(MASK_LEFT_HIP_SIDE, -30 * factor, frameCount);
  	sfPoseGroup(MASK_RIGHT_HIP_SIDE, 30 * factor, frameCount);
	sfWaitFrame(frameCount);
	frameCount = 9;
	sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
	sfWaitFrame(frameCount);
	sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
	sfPoseGroup(MASK_LEFT_HIP_SIDE, 0, frameCount);
  	sfPoseGroup(MASK_RIGHT_HIP_SIDE, 0, frameCount);
	sfWaitFrame(frameCount);
	sfWaitFrame(frameCount);
}

void jump_right() {
	frameCount = 30;
  	sfPoseGroup(MASK_LEFT_FOOT_SIDE, 30 * factor, frameCount);
  	sfPoseGroup(MASK_RIGHT_FOOT_SIDE, -30 * factor, frameCount);
  	sfPoseGroup(MASK_LEFT_HIP_SIDE, 30 * factor, frameCount);
  	sfPoseGroup(MASK_RIGHT_HIP_SIDE, -30 * factor, frameCount);
	sfWaitFrame(frameCount);
	frameCount = 9;
	sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
	sfWaitFrame(frameCount);
	sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
	sfPoseGroup(MASK_LEFT_HIP_SIDE, 0, frameCount);
  	sfPoseGroup(MASK_RIGHT_HIP_SIDE, 0, frameCount);
	sfWaitFrame(frameCount);
	sfWaitFrame(frameCount);
}

void main() {
  int i;
  int cycle_number;
  int mode;
  frameCount = 1;
  pitStop = 0;
  while (pitStop == 0) sfWaitFrame(1); // waithing to change parameters
  cycle_number = jump_mode_local /100;
  factor = (jump_mode_local % 100) / 10;
  if (factor == 0) factor = 10;
  for (i=0; i < cycle_number; i++){
    mode = jump_mode_local % 10;
    if (mode == 1) jump_forward();
    if (mode == 2) jump_backward();
    if (mode == 3) jump_left();
    if (mode == 4) jump_right();
    if (mode == 5) jump_turn_CCW();
    if (mode == 6) jump_turn_CW();
    }
  jump_mode_local = 0;
}