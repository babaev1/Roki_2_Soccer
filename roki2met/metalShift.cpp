#include <roki2met.h>

#define MASK_HIP (MASK_LEFT_HIP | MASK_RIGHT_HIP)
#define MASK_KNEE (MASK_LEFT_KNEE | MASK_RIGHT_KNEE | MASK_LEFT_KNEE_BOT | MASK_RIGHT_KNEE_BOT)
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)

int walkLeft;
int walkStandAngle; //Угол, когда стоим, т.е. степень присогнутости ног
int walkSidingAngle; //Угол перекатывания в сторону
int walkFootAddon;   //Дополнительный угол на стопы при перекатывании
int walkLegUpAngle;  //Величина подъема неактивной ноги
int walkTorsoAngle;  //Угол вращения торсом при хотьбе
int walkLegForward;  //Угол вынесения ноги вперед
int walkHipBackward; //Угол отнесения ноги назад
int walkLegDnAngle;  //Угол опускания ноги

int walkMoveSide;    //Смещение в сторону на каждом шаге
int walkMoveRotate;  //Поворот на каждом шаге
int walkMoveForward; //Смещение вперед-назад на каждом шаге

int frame;
int angle;
int angleFoot;
int maskSingleAngle;
int maskDoubleAngle;

//Вытянуться в 0
//     [rRr]
//     [lLl]
void stand0() {
  frame = 50;
  sfPoseGroup( MASK_ALL & ~(MASK_HIP | MASK_FOOT_FRONT | MASK_KNEE), 0, frame );
  //Сгиб бедра и стопы
  sfPoseGroup( MASK_HIP | MASK_FOOT_FRONT, walkStandAngle, frame );
  //Устанавливаем угол сгиба колена
  sfPoseGroup( MASK_KNEE, walkStandAngle, frame );
  sfWaitFrame( frame );
  }

int pushFrame;
int slideFrame;
int legUpFrame;

int downFrame;
int downSideFrame;


//Толчок и поднятие ноги
void pushAndUp() {
  //Выпрямить пассивную ногу
  sfPoseGroup( walkLeft ? (MASK_LEFT_HIP | MASK_LEFT_KNEE | MASK_LEFT_FOOT_FRONT | MASK_LEFT_KNEE_BOT) : (MASK_RIGHT_HIP | MASK_RIGHT_KNEE | MASK_RIGHT_FOOT_FRONT | MASK_RIGHT_KNEE_BOT), 0, pushFrame );
//  angle = walkLeft ? -walkSidingAngle : walkSidingAngle;
//  angleFoot = walkLeft ? -(walkSidingAngle + walkFootAddon) : (walkSidingAngle + walkFootAddon);

//  sfPoseGroup( MASK_RIGHT_HIP_SIDE, angle, slideFrame );
//  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, angleFoot, slideFrame );
//  sfPoseGroup( MASK_LEFT_HIP_SIDE, -angle, slideFrame );
//  sfPoseGroup( MASK_LEFT_FOOT_SIDE, -angleFoot, slideFrame );
  sfWaitFrame( pushFrame );
  
  //Поднимаем ногу
  maskSingleAngle = walkLeft ? (MASK_LEFT_HIP | MASK_LEFT_FOOT_FRONT | MASK_LEFT_KNEE | MASK_LEFT_KNEE_BOT) : (MASK_RIGHT_HIP | MASK_RIGHT_FOOT_FRONT | MASK_RIGHT_KNEE | MASK_RIGHT_KNEE_BOT);
  sfPoseGroup( maskSingleAngle, walkLegUpAngle, legUpFrame );

  sfWaitFrame( legUpFrame );
  }
  
//Поднятие ноги и перекатывание
void legUp() {
//  angle = walkLeft ? -walkSidingAngle : walkSidingAngle;
//  angleFoot = walkLeft ? -(walkSidingAngle + walkFootAddon) : (walkSidingAngle + walkFootAddon);
//  sfPoseGroup( MASK_RIGHT_HIP_SIDE, angle, legUpFrame );
//  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, angleFoot, legUpFrame );
//  sfPoseGroup( MASK_LEFT_HIP_SIDE, -angle, legUpFrame );
//  sfPoseGroup( MASK_LEFT_FOOT_SIDE, -angleFoot, legUpFrame );
  
  //Поднимаем ногу
  maskSingleAngle = walkLeft ? (MASK_LEFT_HIP | MASK_LEFT_FOOT_FRONT | MASK_LEFT_KNEE | MASK_LEFT_KNEE_BOT) : (MASK_RIGHT_HIP | MASK_RIGHT_FOOT_FRONT | MASK_RIGHT_KNEE | MASK_RIGHT_KNEE_BOT);
  sfPoseGroup( maskSingleAngle, walkLegUpAngle, legUpFrame );

  
  sfWaitFrame( legUpFrame );
  }

//Опускание ноги
void putDown() {
  maskSingleAngle = walkLeft ? (MASK_LEFT_HIP | MASK_LEFT_FOOT_FRONT | MASK_LEFT_KNEE | MASK_LEFT_KNEE_BOT) : (MASK_RIGHT_HIP | MASK_RIGHT_FOOT_FRONT | MASK_RIGHT_KNEE | MASK_RIGHT_KNEE_BOT);
  sfPoseGroup( maskSingleAngle, walkStandAngle, downFrame );

  
  sfWaitFrame( downFrame );
  }

void init() {
  pushFrame = 20;
  slideFrame = 15;
  legUpFrame = 15;
  downFrame = 15;
  downSideFrame = 10;
  walkStandAngle = 800;
  walkSidingAngle = 200;
  walkLegUpAngle = 1400;
  walkTorsoAngle = 800;
  walkLegForward = 0;
  walkHipBackward = 0;
  walkLegDnAngle = 500;
  walkMoveRotate = -200;
  walkMoveForward = 800;
  }

void main() {
  init();
  
  stand0();
  
  //Подготавливаем и запускаем аккумулятор
  sfAccumSetup4( &leftKneeCurrent, &svImuAccX, &rightKneeCurrent, &leftKneeTarget );
  sfAccumTrigger();

  pushAndUp();
  
  while(1) {

    putDown();
    
    int sign = walkLeft ? -1 : 1;
    while( svImuAccX * sign < 0 )
      sfWaitFrame( 1 );
    
    if( walkLeft )
      walkLeft = 0;
    else
      walkLeft = 1;
    
    legUp();
    }
  }

