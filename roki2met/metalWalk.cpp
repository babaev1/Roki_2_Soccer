#include <roki2met.h>


#define MASK_LEG_LEFT (MASK_LEFT_PELVIC | MASK_LEFT_HIP_SIDE | MASK_LEFT_HIP | MASK_LEFT_KNEE | MASK_LEFT_FOOT_SIDE | MASK_LEFT_FOOT_FRONT)
#define MASK_LEG_RIGHT (MASK_RIGHT_PELVIC | MASK_RIGHT_HIP_SIDE | MASK_RIGHT_HIP | MASK_RIGHT_KNEE | MASK_RIGHT_FOOT_SIDE | MASK_RIGHT_FOOT_FRONT)

#define MASK_LEG_LEFT_NO_HIP (MASK_LEFT_PELVIC | MASK_LEFT_HIP_SIDE | MASK_LEFT_KNEE | MASK_LEFT_FOOT_SIDE | MASK_LEFT_FOOT_FRONT)
#define MASK_LEG_RIGHT_NO_HIP (MASK_RIGHT_PELVIC | MASK_RIGHT_HIP_SIDE | MASK_RIGHT_KNEE | MASK_RIGHT_FOOT_SIDE | MASK_RIGHT_FOOT_FRONT)

int walkLeft;        //Флаг активной ноги. Когда 1 - поднимаем и выносим вперед левую ногу, когда 0 - правую
int walkSign;        //Переменная направления вращения торса
int walkBaseHeight;  //Базовая высота промежности от земли. Определяет полусогнутость ног, когда робот стоит неподвижно
int walkLegUp;       //Величина подъема ноги над землей при переносе
int walkPush;
int imuXOffset;      //Смещение налона при ветикальном положении
int walkFront;       //Величина шага вперед
int walkBack;
int tiltForward;     //Дополнительный наклон вперед
int walkType;
int drawdown;        //Величина просадки при установке на пол
int forwardFactor;   //Фактор зависимости постановки ноги вперед от наклона вперед
int forwardPelvic;   //Величина подкручивания тазом
int tiltAddon;
int flag;
int veloForward;
int isWalk;


void humoConfig() {
  svHumoModelDistPelvic = 25;
  svHumoModelLengthPelvicHip = 21;
  svHumoModelLengthHipHip = 48;
  svHumoModelLengthHipKnee = 64;
  svHumoModelLengthKneeFoot = 68;
  svHumoModelLengthFootFoot = 48;
  svHumoModelDistFootFloor = 25;
  
  svHumoModelFootLength = 114;
  svHumoModelFootWidth = 75;
  svHumoModelFootCenterBack = 57;
  svHumoModelFootCenterInner = 23;
  sfHumoModelSetup();
  
  walkBaseHeight = 243; //Базовая высота
  walkLegUp = 18; //Высота подъема ноги при хотьбе
  walkPush = 2; //Величина толчка при старте
  imuXOffset = 135;
  drawdown = 5; //Величина просадки при установке на пол
  forwardFactor = 0;
  tiltAddon = 200;
  }


void legFromModel( int mask, int frameCount, int delay ) {
  sfPoseGroupDelay( mask & (MASK_LEFT_HIP_SIDE | MASK_RIGHT_HIP_SIDE), svHumoModelAngHipSide, frameCount, delay );
  sfPoseGroupDelay( mask & (MASK_LEFT_HIP | MASK_RIGHT_HIP), svHumoModelAngHip + tiltForward, frameCount, delay );
  sfPoseGroupDelay( mask & (MASK_LEFT_KNEE | MASK_RIGHT_KNEE), svHumoModelAngKnee, frameCount, delay );
  sfPoseGroupDelay( mask & (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT), svHumoModelAngFoot, frameCount, delay );
  sfPoseGroupDelay( mask & (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE), svHumoModelAngFootSide, frameCount, delay );
  }

void legFromModelAndWait( int mask, int frameCount ) {
  legFromModel( mask, frameCount, 0 );
  sfWaitFrame( frameCount );
  }

void legFromModelLinear( int mask, int frameCount ) {
  sfPoseGroupLin( mask & (MASK_LEFT_HIP_SIDE | MASK_RIGHT_HIP_SIDE), svHumoModelAngHipSide, frameCount );
  sfPoseGroupLin( mask & (MASK_LEFT_HIP | MASK_RIGHT_HIP), svHumoModelAngHip + tiltForward, frameCount );
  sfPoseGroupLin( mask & (MASK_LEFT_KNEE | MASK_RIGHT_KNEE), svHumoModelAngKnee, frameCount );
  sfPoseGroupLin( mask & (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT), svHumoModelAngFoot, frameCount );
  sfPoseGroupLin( mask & (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE), svHumoModelAngFootSide, frameCount );
  }

int deltaFront;
int frontStep;
int heightStep;
int index;
int posFront;
int posHeight;

void walking() {
  while(1) {
    //Ожидаем, пока скорость не станет ненулевой
    while( veloForward == 0 )
      sfWaitFrame( 1 );
    
    //Выровнять по 4
    veloForward = (veloForward >> 2) << 2;
    
    //Оттолкнуться
    walkLeft = 1;
    walkSign = -1;
    walkBack = 0;
  
    //Выполнить толчок
    tiltForward = tiltAddon;
    sfHumoModelFootAtFloor( 0, 0, walkBaseHeight + walkPush );
    legFromModelAndWait( MASK_LEG_RIGHT, 10 );
  
    //Подготавливаем и запускаем аккумулятор
    if( flag == 0 ) {
      sfAccumSetup4( &leftHipTarget, &leftKneeTarget, &leftFootFrontTarget, &posFront );
      sfAccumTrigger();
      flag = 1;
      }
    
    isWalk = 2;
    walkFront = 0;
  
    while( isWalk ) {
      //Ожидаем перекатывания на другую сторону
      while( (svImuAccX - imuXOffset) * walkSign < 0 )
        sfWaitFrame( 1 );
    
      //Смена ноги
      if( walkLeft ) {
        walkLeft = 0;
        walkSign = 1;
        //Проверим, не зависла ли левая нога
        if( leftKneePwm > 0 ) return;
        }
      else {
        walkLeft = 1;
        walkSign = -1;
        //Проверим, не зависла ли правая нога
        if( rightKneePwm > 0 ) return;
        }

      tiltForward = tiltAddon + walkFront * 4;
    
      //На опорной ноге движемся вперед (или назад)
      sfHumoModelFootAtFloor( -walkFront, 0, walkBaseHeight );
      legFromModel( walkLeft == 0 ? MASK_LEG_LEFT : MASK_LEG_RIGHT, 30, 2 );
      //sfPoseGroupDelay( MASK_LEFT_PELVIC, walkLeft ? forwardPelvic : -forwardPelvic, 15, 0 );
      //sfPoseGroupDelay( MASK_RIGHT_PELVIC, walkLeft ? -forwardPelvic : forwardPelvic, 15, 0 );
    
#ifdef MMM      
      deltaFront = walkFront + walkBack;
      frontStep = 1024 / 32;
      heightStep = 1024 / 8;
      index = 0;
      int i;
      for( i = 0; i < 32; i++ ) {
        index += frontStep;
        posFront = sfMathTableSin( index, deltaFront );
        //На первых четырех шага ногу поднимаем, на последних - опускаем
        if( i < 4 ) posHeight = sfMathTableSin( (i + 1) * heightStep, walkLegUp );
        else if( i < 28 ) posHeight = walkLegUp;
        else posHeight = walkLegUp - sfMathTableSin( (i - 27) * heightStep, walkLegUp );
        
        sfHumoModelFootAtFloor( -walkBack + posFront, 0, walkBaseHeight - posHeight );
        legFromModelLinear( walkLeft ? MASK_LEG_LEFT : MASK_LEG_RIGHT, 1 );
        sfWaitFrame( 1 );
        }
#endif
    
      //Поднимаем ногу
      sfHumoModelFootAtFloor( -walkBack, 0, walkBaseHeight - walkLegUp );
      legFromModel( walkLeft ? MASK_LEG_LEFT_NO_HIP : MASK_LEG_RIGHT_NO_HIP, 12, 0 );
     // legFromModelAndWait( walkLeft ? MASK_LEG_LEFT : MASK_LEG_RIGHT, 10 );
    
      //Выносим вперед
      sfHumoModelFootAtFloor( walkFront, 0, walkBaseHeight - walkLegUp );
      legFromModel( walkLeft ? MASK_LEFT_HIP : MASK_RIGHT_HIP, 24, 0 );
      sfWaitFrame( 8 );

      legFromModel( walkLeft ? MASK_LEG_LEFT_NO_HIP : MASK_LEG_RIGHT_NO_HIP, 12, 0 );
      sfWaitFrame( 12 );
      //      legFromModelAndWait( walkLeft ? MASK_LEG_LEFT : MASK_LEG_RIGHT, 10 );
    
      //Опускаем ногу
      sfHumoModelFootAtFloor( walkFront, 0, walkBaseHeight );
      legFromModelAndWait( walkLeft ? MASK_LEG_LEFT : MASK_LEG_RIGHT, 12 );
    
      walkBack = walkFront;
      if( walkFront == 0 ) isWalk--;
      
      //Постепенный разгон и торможение
      if( walkFront < veloForward ) walkFront += 2;
      if( walkFront > veloForward ) walkFront -= 2;
      }
    }
  }


void main() {
  //Согнуть руги в локтях
  sfPoseGroup( MASK_LEFT_CLAVICLE | MASK_RIGHT_CLAVICLE, -1400, 100 );
  sfPoseGroup( MASK_LEFT_SHOULDER | MASK_RIGHT_SHOULDER, 100, 100 );
  sfPoseGroup( MASK_LEFT_ELBOW | MASK_RIGHT_ELBOW, 5000, 100 );
  sfPoseGroup( MASK_HEAD_ROTATE | MASK_HEAD_TILT, 0, 100 );
  sfWaitFrame( 100 );

  humoConfig();
  veloForward = 4;
  
  while(1) {
    //Базовая высота
    tiltForward = tiltAddon;
    sfHumoModelFootAtFloor( 0, 0, walkBaseHeight );
    legFromModelAndWait( MASK_LEG_LEFT | MASK_LEG_RIGHT, 50 );
    
    //Ожидаем, пока робота не поставят на пол
    while( leftKneePwm > 0 || rightKneePwm > 0 )
      sfWaitFrame( 1 );
    
    //Сделать просадку
    sfHumoModelFootAtFloor( 0, 0, walkBaseHeight - drawdown );
    legFromModelAndWait( MASK_LEG_LEFT | MASK_LEG_RIGHT, 10 );
    //Восстановить восстановление на базовый уровень
    sfHumoModelFootAtFloor( 0, 0, walkBaseHeight );
    legFromModelAndWait( MASK_LEG_LEFT | MASK_LEG_RIGHT, 30 );
    
    walking();
    }
  }

