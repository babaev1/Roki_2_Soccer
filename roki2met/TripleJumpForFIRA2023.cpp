// This is motion 'TripleJumpForFIRA2023' translated from json file 

#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT )
//#define MASK_KNEE_BOT ( MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_FOOT_SIDE (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)
#define MASK_SHOULDER (MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER)
#define MASK_CLAVICLE (MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE)
#define MASK_ELBOW (MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW)

int restart_flag;
int frameCount;
float factor;
int tuner;
int pitStop;
int hip_at_landing;

void page_0() {
	frameCount = 250;
  	frameCount *= factor;
	sfPoseGroup(MASK_ALL, 0, frameCount);
	sfWaitFrame(frameCount);
}

void page_1() {
	frameCount = 30;
  	frameCount *= factor;
	sfPoseGroup(MASK_FOOT_FRONT, 3072, frameCount);
	sfPoseGroup(MASK_KNEE, 3072, frameCount);
  	//sfPoseGroup(MASK_KNEE, 0, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 5650, frameCount);
	sfPoseGroup(MASK_HIP, 3456, frameCount);
	sfPoseGroup(MASK_SHOULDER, 460, frameCount);
	sfPoseGroup(MASK_CLAVICLE, -1022, frameCount);
  	sfPoseGroup(MASK_CLAVICLE, -3000, frameCount);
	sfPoseGroup(MASK_HEAD_TILT, -921, frameCount);
  	sfPoseGroup(MASK_HEAD_TILT, 4000, frameCount);
	sfWaitFrame(frameCount);
}

void page_2() {
	frameCount = 30; //12;
  	frameCount *= factor;
	//sfPoseGroup(MASK_FOOT_FRONT, 3394, frameCount);
  	sfPoseGroup(MASK_FOOT_FRONT, 2680 + tuner, frameCount);
	//sfPoseGroup(MASK_HIP, 3900, frameCount);
  	sfPoseGroup(MASK_HIP, 4100, frameCount);
  	//sfPoseGroup(MASK_KNEE, 3400, frameCount);
	sfWaitFrame(frameCount);
}

void page_3(int i) {     
	frameCount = 1;  		// 15
  	//frameCount *= factor;
	sfPoseGroupLin(MASK_FOOT_FRONT, 3072 + i, frameCount);
	sfPoseGroupLin(MASK_KNEE, 1920 + i, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 1920, frameCount);
	sfPoseGroupLin(MASK_HIP, 1536 + i, frameCount);
	sfPoseGroupLin(MASK_CLAVICLE, 2304 + i, frameCount);
  	sfPoseGroup(MASK_HEAD_TILT, -921 + i, frameCount);
	sfWaitFrame(frameCount);
}

void page_4(int i) {
	frameCount = 1; //8;
  	//frameCount *= factor;
	sfPoseGroupLin(MASK_FOOT_SIDE, 921 + i, frameCount);
	sfPoseGroupLin(MASK_FOOT_FRONT, -307 + i, frameCount);
  	sfPoseGroupLin(MASK_FOOT_FRONT, -907 + i, frameCount);
	sfPoseGroupLin(MASK_KNEE, 153 + i, frameCount);
  	//sfPoseGroup(MASK_KNEE, 0, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 0, frameCount);
	sfPoseGroupLin(MASK_HIP, -1996 + i, frameCount);
  	//sfPoseGroupLin(MASK_HIP, 0 + i, frameCount);
	sfPoseGroupLin(MASK_CLAVICLE, 3840 + i, frameCount);
  	sfPoseGroup(MASK_HEAD_TILT, -921 + i, frameCount);
	sfWaitFrame(frameCount);
}
void page_5(int i) {
	frameCount = 1;		// 2
  	frameCount *= factor;
	sfPoseGroupLin(MASK_FOOT_SIDE, 1536 + i, frameCount);
	sfPoseGroupLin(MASK_FOOT_FRONT, -1474 + i, frameCount);
  	//sfPoseGroup(MASK_FOOT_FRONT, -2000, frameCount);
	sfPoseGroupLin(MASK_KNEE, 0 + i, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 6100, frameCount);
	sfPoseGroupLin(MASK_HIP, -2304 + i, frameCount);
  	//sfPoseGroupLin(MASK_HIP, -500 + i, frameCount);
	sfPoseGroupLin(MASK_ELBOW, 622 + i, frameCount);
	//sfPoseGroupLin(MASK_CLAVICLE, 5938 + i, frameCount);
  	sfPoseGroupLin(MASK_CLAVICLE, 3840 + i, frameCount);
  	sfPoseGroup(MASK_HEAD_TILT, -921 + i, frameCount);
	sfWaitFrame(frameCount);
}

void page_6(int i) {
	frameCount = 1;		// 1
  	//frameCount *= factor;
	sfPoseGroupLin(MASK_FOOT_SIDE, 0 + i, frameCount);
	sfPoseGroupLin(MASK_FOOT_FRONT, 3429 + i, frameCount);
  	//sfPoseGroup(MASK_FOOT_FRONT, 3000, frameCount);
	sfPoseGroupLin(MASK_KNEE, 3500 + i, frameCount);
  	//sfPoseGroup(MASK_KNEE, 400, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 6100, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 3400, frameCount);
	sfPoseGroupLin(MASK_HIP, 5600 + i, frameCount);
  	//sfPoseGroup(MASK_HIP, 3500, frameCount);
	sfPoseGroupLin(MASK_ELBOW, -921 + i, frameCount);
	sfPoseGroupLin(MASK_CLAVICLE, -4094 + i, frameCount);
  	sfPoseGroup(MASK_HEAD_TILT, 4000+ i, frameCount);
	sfWaitFrame(frameCount);
}

void page_6_1(int i) {
	frameCount = 1;
  	frameCount *= factor;
	sfPoseGroupLin(MASK_FOOT_SIDE, 0, frameCount);
	sfPoseGroupLin(MASK_FOOT_FRONT, 3429, frameCount);
  	//sfPoseGroup(MASK_FOOT_FRONT, 3000, frameCount);
	sfPoseGroupLin(MASK_KNEE, 3500, frameCount);
  	//sfPoseGroup(MASK_KNEE, 400, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 6100, frameCount);
  	//sfPoseGroup(MASK_KNEE_BOT, 3400, frameCount);
	sfPoseGroupLin(MASK_HIP, 5191, frameCount);
  	sfPoseGroupLin(MASK_HIP, 3500, frameCount);
	sfPoseGroupLin(MASK_ELBOW, -921, frameCount);
	sfPoseGroupLin(MASK_CLAVICLE, -3583, frameCount);
	sfWaitFrame(frameCount);
}

void page_7() {
	frameCount = 2;
  	//while(leftKneeCurrent < 3420 ) sfWaitFrame(1);
  	sfWaitFrame(frameCount);
}

void page_8_(int i) {
	frameCount = 1;		// 6
  	frameCount *= factor;
	sfPoseGroupLin(MASK_FOOT_FRONT, 3993 + i, frameCount);
  	sfPoseGroupLin(MASK_FOOT_FRONT, 3000 + i, frameCount);
	//sfPoseGroup(MASK_HIP, 3072, frameCount);
  	sfPoseGroupLin(MASK_HIP, 5600 + i, frameCount);
  	sfPoseGroupLin(MASK_HIP, 5000 + i, frameCount);
  	sfPoseGroupLin(MASK_HIP, 4800 + i, frameCount);
	sfPoseGroupLin(MASK_ELBOW, -921 + i, frameCount);
	sfPoseGroupLin(MASK_CLAVICLE, -4094 + i, frameCount);
  	sfPoseGroup(MASK_HEAD_TILT, 4000+ i, frameCount);
	sfWaitFrame(frameCount);
}
void page_8(int i) {
	frameCount = 1;		// 6
  	frameCount *= factor;
	sfPoseGroupLin(MASK_FOOT_FRONT, 3993 , frameCount);
  	sfPoseGroupLin(MASK_FOOT_FRONT, 3000, frameCount);
	//sfPoseGroup(MASK_HIP, 3072, frameCount);
  	sfPoseGroupLin(MASK_HIP, 4800 + hip_at_landing , frameCount);
	sfPoseGroupLin(MASK_ELBOW, -921 , frameCount);
	sfPoseGroupLin(MASK_CLAVICLE, -4094 , frameCount);
  	sfPoseGroup(MASK_HEAD_TILT, 4000, frameCount);
	sfWaitFrame(frameCount);

}

void page_9() {
	frameCount = 175;
  	frameCount *= factor;
	sfWaitFrame(frameCount);

}

void page_9_1() {
	frameCount = 175;
  	frameCount *= factor;
  	sfPoseGroup(MASK_HIP, 5072, frameCount);
	sfWaitFrame(frameCount);

}

void page_10(){
  frameCount = 75;
  frameCount *= factor;
  sfPoseGroup( MASK_FOOT_FRONT, 3072, frameCount );
  sfPoseGroup( MASK_KNEE, 3072, frameCount );
  sfPoseGroup( MASK_HIP, 3456, frameCount );
  sfPoseGroup( MASK_ELBOW, 0, frameCount );
  sfPoseGroup( MASK_CLAVICLE, -1022, frameCount );

  sfWaitFrame( frameCount );
}

void main() {
  int i;
  int jump;
  tuner = 0;
  factor = 0.9; //0.9
  hip_at_landing = 0;
  /*
    restart_flag = 0;
    restart_flag = 1;
    sfWaitFrame(100);                    // waithing 1sec to change parameters
    svButtonRight = SV_SLOT_INACTIVE;
    svButtonLeft = SV_SLOT_INACTIVE;
    while (svButtonPress != SV_BUTTON_RIGHT_PRESS) sfWaitFrame(1); // waiting to press right button
    svButtonRight = SV_SLOT_RESTART_RUN;
    svButtonLeft = SV_SLOT_RELAX;
    restart_flag = 0;
*/
  while (pitStop == 0) sfWaitFrame(1); // waithing to change parameters
  for( jump = 0; jump < 3; i++ ){
	page_1();
	page_2();

	for( i = 0; i < 15 * factor; i++ ) page_3(i);
	for( i = 0; i < 8 * factor; i++ ) page_4(i);
	for( i = 0; i < 3 * factor; i++ ) page_5(i);
	for( i = 0; i < 2 * factor; i++ ) page_6(i);
  	//page_6_1();
  	//page_7();
  
  	//page_3();
	//page_4();
	//page_5();
	//page_6();
  
	page_7();
	for( i = 0; i < 6 * factor; i++ ) page_8(i);
	page_9();
  	page_9_1();
  	page_0();
	//page_10();
    	sfWaitFrame( 1500 );
    }
}