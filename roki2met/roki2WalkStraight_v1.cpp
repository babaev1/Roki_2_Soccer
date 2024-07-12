/*
  Проект "Roki-2"
  Автор
    Зубр ОВК
  Описание
    Скрипт движок ходьбы по методу Азера на прямых ногах
*/
#include <roki2met.h>

int slowWalk;

float stepLength; // = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
float sideLength; // = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
float rotation; // = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.


float selfMotionShiftCorrectionX; // = -self.glob.params['MOTION_SHIFT_TEST_X'] / 21
float selfMotionShiftCorrectionY; // = -self.glob.params['MOTION_SHIFT_TEST_Y'] / 21
float selfFirstStepYield; // = self.glob.first_step_yield
float selfCycleStepYield; // = self.glob.cycle_step_yield
float selfSideStepRightYield; // = self.glob.side_step_right_yield
float selfSideStepLeftYield; // = self.glob.side_step_left_yield
//        self.imu_drift_speed = math.radians(self.glob.params['IMU_DRIFT_IN_DEGREES_DURING_6_MIN_MEASUREMENT'])/360
int   selfFirstLegIsRightLeg; // = True
int   selfInitPoses; // = 400//self.simThreadCycleInMs

int   selfExitFlag; // = 0
int   selfFallingFlag; // = 0
int   selfNeckPan; // = 0
float rotationYieldRight;
float rotationYieldLeft;

int   framestep;

//        self.body_euler_angle ={}
//        self.head_quaternion = None
//        self.activePose = []
float xtr; // = 0
float ytr; // = -self.d10   #-53.4
float ztr; // = -self.gaitHeight

float xr; // = 0
float yr; // = 0
float zr; // = -1
float wr; // = 0

float xtl; // = 0
float ytl; // = self.d10   # 53.4
float ztl; // = -self.gaitHeight

float xl; // = 0
float yl; // = 0
float zl; // = -1
float wl; // = 0

float e10;
float d10;

float amplitude;
float alpha01;
int fr1;
int fr2;
float gaitHeight;
float stepHeight;
float correctedStepLenght;
float correctedStepLenghtHalf;
float correctedSideLenght;
float correctedSideLenghtHalf;

float ztr0;
float ztl0;

float stepZtr;
float stepZtl;
float stepYtr;
float stepYtl;

#define STEP_FIRST 0
#define STEP_LAST  1
#define STEP_OTHER 2
int stepType;
int fps;
int frames_per_cycle;
int relaxFootR;
int relaxFootL;
//relaxFootR = 0;
//relaxFootL = 0;
float bodyTiltAtWalk;
float solyLandingSkew;
int segments_num;
int segments[100];
int segments_sum;
int j;
int i;

void parabaloid_list_creator(){
  segments_num = fr2 / framestep - 2;
  segments_sum = 0;
  for( j = 0; j < segments_num; j ++ ) {
    if (j == 0 || j ==  segments_num -1){
      segments[j] = 1;
      }
    else if(j * 2 < segments_num){
      segments[j] = 1;
      for (i = 0; i < j; i++){
          segments[j] *= 2;
          }
      }
    else{
      segments[j] = 1;
      for (i = 0; i < (segments_num - j -1); i++){
          segments[j] *= 2;
          }
      }
    segments_sum += segments[j];
    }
}

float test1;
float test2;
float test3;
float alpha6;
float alpha7;
float alpha9;
float alpha10;
float IkEncA5, IkEncA6, IkEncA7, IkEncA8, IkEncA9, IkEncA10;
float IkOutPresent;

void straight_leg_IK(float xt, float yt, float x, float y, float w){
  IkOutPresent = 0;
  if (svIkLimA5min <= w && w <= svIkLimA5max){
    alpha6 = sfFMathASin((yt + svIkA5 + svIkB5 + svIkA10)/(svIkA6 + svIkA7 + svIkA8 + svIkA9));
    test1 = yt;
    test3 = (yt + svIkA5 + svIkB5 + svIkA10)/(svIkA6 + svIkA7 + svIkA8 + svIkA9);
    if (svIkLimA6min <= alpha6 && alpha6 <= svIkLimA6max ){
      test2 = xt/(svIkA7 + svIkA8);
      alpha7 = sfFMathASin(xt/(svIkA7 + svIkA8));
      if (svIkLimA7min <= alpha7 && alpha7 <= svIkLimA7max){
        alpha9 = sfFMathASin(x) - alpha7;
        if (svIkLimA9min <= alpha9 && alpha9 <= svIkLimA9max){
          alpha10 = -sfFMathASin(y) + alpha6;
          if (svIkLimA10min <= alpha10 && alpha10 <= svIkLimA10max){
            IkEncA5 = w * 2607.6;		// number of ticks per radian
            IkEncA6 = alpha6 * 2607.6;
            IkEncA7 = alpha7 * 2607.6;
            IkEncA8 = 0;				// knee is straight
            IkEncA9 = alpha9 * 2607.6;
            IkEncA10 = alpha10 * 2607.6;
            IkOutPresent = 1;
            }
          }
        }
      }
    } 
  }

void setup() {
  rotationYieldRight = 0.23;
  rotationYieldLeft = 0.23;
  //Установка параметров модели для инверсной кинематики
  svIkA5 = 40.2; // мм расстояние от оси симметрии до оси сервы 5
  svIkB5 = 0;    // мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
  svIkC5 = 0;    // мм расстояние от оси сервы 6 до нуля Z по вертикали
  svIkA6 = 0;    // мм расстояние от оси сервы 6 до оси сервы 7
  svIkA7 = 99;   // мм расстояние от оси сервы 7 до оси сервы 8
  svIkA8 = 99;   // мм расстояние от оси сервы 8 до оси сервы 9
  svIkA9 = 0;    // мм расстояние от оси сервы 9 до оси сервы 10
  svIkA10 = 13.7;  // мм расстояние от оси сервы 10 до центра стопы по горизонтали
  svIkB10 = 23.8;  // мм расстояние от оси сервы 10 до низа стопы
  svIkC10 = 0;   // мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
  
  
  e10 = 55;      // мм половина длины стопы
  d10 = 62;      // расстояние по Y от центра стопы до оси робота
  
  //Ограничения движения моторов в градусах относительно нулевого положения
  svIkLimA5min = -90 * MATH_RAD_PER_DEGREE;
  svIkLimA5max = 90 * MATH_RAD_PER_DEGREE;
  
  //инв
  svIkLimA6min = -108 * MATH_RAD_PER_DEGREE;
  svIkLimA6max = 15 * MATH_RAD_PER_DEGREE;
  
  svIkLimA7min = -130 * MATH_RAD_PER_DEGREE;
  svIkLimA7max = 50 * MATH_RAD_PER_DEGREE;
  
  //инв
  svIkLimA8min = -180 * MATH_RAD_PER_DEGREE;
  svIkLimA8max = 8 * MATH_RAD_PER_DEGREE;
  
  //инв
  svIkLimA9min = -45 * MATH_RAD_PER_DEGREE;
  svIkLimA9max = 78 * MATH_RAD_PER_DEGREE;
  
  //инв
  svIkLimA10min = -24 * MATH_RAD_PER_DEGREE;
  svIkLimA10max = 24 * MATH_RAD_PER_DEGREE;
  sfIkSetup();

  framestep = 2;
  
  gaitHeight = 180;  // Distance between Center of mass and floor in walk pose
  stepHeight = 38.0; // elevation of sole over floor
  
  selfMotionShiftCorrectionX = 0.0 / 21.0;
  selfMotionShiftCorrectionY = 0.0 / 21.0;

  //self.LIMALPHA[3][1]=0

  ztr0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -223.1
  ztl0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -223.1
  
  zr = zl = -1;
  
  
  selfInitPoses = 10;
  
 // selfTIK2RAD = 0.00058909;
  stepLength = 0.0;    // -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
  sideLength = 0.0;    // -20 - +20. Side step length to right (+) and to left (-)
  rotation = 0;        // -45 - +45 degrees Centigrade per step + CW, - CCW.
  selfFirstLegIsRightLeg = 1; // = True
  selfInitPoses = 20; //self.simThreadCycleInMs

  stepZtr = (ztr0 + gaitHeight) / selfInitPoses;
  stepZtl = (ztl0 + gaitHeight) / selfInitPoses;
  
  bodyTiltAtWalk = 0.04;
  solyLandingSkew = 0.0;
  
  if( slowWalk ) {
    fr1 = 50; //8;           // frame number for 1-st phase of gait ( two legs on floor)
    fr2 = 20; //12;          // frame number for 2-nd phase of gait ( one leg in air)
    amplitude = 110;    // mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
    }
  else {
    fr1 = 8;           // frame number for 1-st phase of gait ( two legs on floor)
    fr2 = 16;          // frame number for 2-nd phase of gait ( one leg in air)
    amplitude = 10;    // mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
    }
  stepYtr = amplitude / 2.0 / selfInitPoses;
  stepYtl = amplitude / 2.0 / selfInitPoses;
//        if self.fr1 == 0:
//            alpha01 = math.pi
//        else:
//            alpha01 = math.pi/self.fr1*2
  alpha01 = MATH_PI;
  if( fr1 != 0 )
    alpha01 = alpha01 * 2 / fr1;
  
  //Длительность шага алгоритма
  //На каждом шаге происходит линейная интерполяция позиций в течение количества
  //фреймов, заданных данной переменной
  //Фактическая длительность шага составляет fps - 1, т.е. она короче на один фрейм
  fps = 8;
  frames_per_cycle = 2;
  }
  

//Вычисление углов серв с помощью обратной кинематики и отправка их в сервы
//Ожидание завершения перемещения
//Перемещение выполняется с помощью линейной интерполяции промежуточных значений
//fps определяет расчетную длительность шага, фактическая длительность на 1 меньше

int computeAlphaForWalkStraight(int fasa, int cycle) {
  //Вычислить значения углов серв для правой ноги
  // fasa = 0 если начальное или конечное движение, fasa = 1,2,3,4 в зависимости от фазы
  straight_leg_IK( xtr, ytr, xr, yr, wr ); 
  if( IkOutPresent ) {
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
      if (fasa == 2){
        if(cycle == 0 || cycle == (fr2 - framestep)) IkEncA6 += stepHeight / 124.0 / 2 * 2607.6;
        if (framestep <= cycle && cycle < fr2 - framestep){
          IkEncA6 += stepHeight / 124.0 * 2607.6;
          IkEncA10 += 0.3 * 2607.6;
          }
        }
      if (fasa == 4){
        if(cycle == 0 || cycle == (fr2 - framestep)) IkEncA6 -= stepHeight / 124.0 / 2 * 2607.6;
        if (framestep <= cycle && cycle < fr2 - framestep){
          IkEncA6 -= stepHeight / 124.0 * 2607.6;
          
          }
        }
    	sfPoseGroupLin( MASK_RIGHT_PELVIC, -IkEncA5, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP_SIDE, -IkEncA6, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP, IkEncA7, fps );
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -IkEncA8, fps );
     if (relaxFootR == 0){
    	sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT, IkEncA9, fps );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE, -IkEncA10, fps );
      }
      else{
        relaxFootR = 0;
        sfFreeGroup( MASK_RIGHT_FOOT_FRONT | MASK_RIGHT_FOOT_SIDE );
        
        }
      }
    else {
      sfPoseGroupLin( MASK_LEFT_PELVIC, IkEncA5, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP_SIDE, -IkEncA6, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP, -IkEncA7, fps );
    	sfPoseGroupLin( MASK_LEFT_KNEE, -IkEncA8, fps );
     if (relaxFootR == 0){
    	sfPoseGroupLin( MASK_LEFT_FOOT_FRONT, IkEncA9, fps );
    	sfPoseGroupLin( MASK_LEFT_FOOT_SIDE, -IkEncA10, fps );
      }
     else{
        relaxFootR = 0;
        sfFreeGroup( MASK_LEFT_FOOT_FRONT | MASK_LEFT_FOOT_SIDE );
        
        }
      }
    }
  //Вычислить значение углов серв для левой ноги
  straight_leg_IK( xtl, -ytl, xl, -yl, wl );
  if( IkOutPresent ) {
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
      if (fasa == 2){
        if(cycle == 0 || cycle == (fr2 - framestep)) IkEncA6 -= stepHeight / 124.0 / 2 * 2607.6;
        if (framestep <= cycle && cycle < fr2 - framestep){
          IkEncA6 -= stepHeight / 124.0 * 2607.6;
          }
        }
      if (fasa == 4){
        if(cycle == 0 || cycle == (fr2 - framestep)) IkEncA6 += stepHeight / 124.0 / 2 * 2607.6;
        if (framestep <= cycle && cycle < fr2 - framestep){
          IkEncA6 += stepHeight / 124.0 * 2607.6;
          IkEncA10 += 0.3 * 2607.6;
          }
        }
    	sfPoseGroupLin( MASK_LEFT_PELVIC, -IkEncA5, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP_SIDE, -IkEncA6, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP, IkEncA7, fps );
    	sfPoseGroupLin( MASK_LEFT_KNEE, -IkEncA8, fps );
     if (relaxFootL == 0){
    	   sfPoseGroupLin( MASK_LEFT_FOOT_FRONT, IkEncA9, fps );
    	   sfPoseGroupLin( MASK_LEFT_FOOT_SIDE, -IkEncA10, fps );
        }
     else{
        relaxFootL = 0;
        sfFreeGroup( MASK_LEFT_FOOT_FRONT | MASK_LEFT_FOOT_SIDE );
        
        }
    }  
    else {
     sfPoseGroupLin( MASK_RIGHT_PELVIC, IkEncA5, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP_SIDE, -IkEncA6, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP, -IkEncA7, fps );
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -IkEncA8, fps );   
     if (relaxFootL == 0){
    	  sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT, IkEncA9, fps );
    	  sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE, -IkEncA10, fps );
       }
     else{
        relaxFootL = 0;
        sfFreeGroup( MASK_RIGHT_FOOT_FRONT | MASK_RIGHT_FOOT_SIDE ); 
        
        }
       }
      }
  //Ожидать когда движение завершится
  sfWaitFrame( framestep );
  return 1;
}

//Проверить падение
//void testDrop() {
//  if( sfAbs(svImuAccX) > 50000 || sfAbs(svImuAccZ) > 50000 ) {
    //Произошло падение
    //Все расслабить
    //sfFreeGroup( MASK_ALL );
    //Подождать 1 сек
//    sfWaitFrame(100);
    //Перейти к вставанию
//    sfStartSlot( sfSlotIndex("roki3StandUp.cpp") );
//    }
//  }

//Установить начальную позу на прямых ногах
void walkStraightInitialPose() {
  int pose_taking_cycles = 20;
  xtr = xtl = 0;
  for( j = 0; j < pose_taking_cycles; j++ ) {
    //Задать положение стоп для двух ног
    //ztr = ztr0 + j * (-ztr0 - gaitHeight) / pose_taking_cycles;
    //ztl = ztr;
    ytr = -d10 - j * amplitude * 2.0 /pose_taking_cycles;
    ytl =  ytr + 2.0 * d10;
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalkStraight(0, j);
    }
  }

//Установить исходную позу (ровный на высоте ztr0)
void walkStraightFinalPose() {
  for( j = 0; j < selfInitPoses; j++ ) {
    //Задать положение стоп двух ног
    xtr = ytr = 0;
    wr = wl = 0;
    ytr = -d10 + (selfInitPoses - (j+1)) * stepYtr;
    ytl =  d10 + (selfInitPoses - (j+1)) * stepYtl;
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalkStraight(0, j);
    }
  }

float xtl0;
float xtr0;
float dx0Typical;
float dy0Typical;
float correctedRotation;
float s;
float fase_offset;
float xtl_plan;
float xtr_plan;
float dx, dx0, dx1, dx2, dx4, dy;

//Настроить следующий цикл хотьбы
void walkStraightInit() {
  //Корректируем длину шага и движение в сторону
  correctedStepLenght = stepLength + selfMotionShiftCorrectionX;
  correctedSideLenght = sideLength - selfMotionShiftCorrectionY;
  correctedSideLenghtHalf = correctedSideLenght / 2;
  
  //Корректируем поворот
  correctedRotation = -rotation *0.25 * 0.23 / ( rotation <= 0 ? rotationYieldRight : rotationYieldLeft);
  dx0Typical = correctedStepLenght / (2.0 * fr1 + fr2) * framestep;
  dy0Typical = correctedSideLenght / (2.0 * fr1 + fr2) * framestep;
  
  //Некоторые константные настройки
  xr = xl = bodyTiltAtWalk;
  yl =  solyLandingSkew;
  yr = -solyLandingSkew;
  fase_offset = 2.8;
  parabaloid_list_creator();
  }

//Настроить следующий цикл хотьбы

//Исполнить фазу 1
void walkStraightPhasa1() {
  ztl = ztr = -gaitHeight;
  for( j = 0; j < fr1; j += framestep ) {
    s = (amplitude / 2 + correctedSideLenghtHalf) * sfMathCos(alpha01 * (j / 2 + fase_offset * framestep));
    ytr = s - d10 + correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    if( stepType == STEP_FIRST ) continue;
    relaxFootL = ((j <= 0) ? 1 : 0);
    xtl_plan = correctedStepLenght * ( 0.5 -  fr1/( 2.0 * fr1 + fr2));
    dx1 = (xtl_plan - xtl) / (fr1 - i) * framestep;
    xtl += dx1;
    xtr += dx1;
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalkStraight(0, j);
    }
  //testDrop();
  }

//Исполнить фазу 3
void walkStraightPhasa3() {
  ztl = ztr = -gaitHeight;
  for( j = 0; j < fr1; j += framestep ) {
    s = (amplitude / 2.0 + correctedSideLenghtHalf) * sfMathCos(alpha01 * ((j +fr1) / 2 + fase_offset * framestep));
    ytr = s - d10 - correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    xtl += dx2;
    xtr += dx2;
    relaxFootR = ((j <= 0) ? 1 : 0);
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalkStraight(0, j);
    }
  //testDrop();
  }
  
float stepRotation;

//Исполнить фазу 2
void walkStraightPhasa2() {
  ztr = -gaitHeight + stepHeight;
  dy = correctedSideLenght * framestep / fr2;
  
  //Из выражения self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
  //выносим за цикл то, что не изменяется rotation/(self.fr2- 2 * framestep)*2
  stepRotation = correctedRotation / (fr2 - 2.0 * framestep) * 2.0;
  xtl_plan = correctedStepLenght *( 0.5 - (fr1 + fr2) / (2.0 * fr1 + fr2));
  xtr_plan = correctedStepLenght / 2.0 + dx0Typical;
  for( j = 0; j < fr2; j += framestep ) {
    dx2 = (xtl_plan - xtl) / (fr2 - j) * framestep;
    if( j == 0 ) {
      xtr += dx2;
      ytr  = s - d10 + dy0Typical;
      }
    else if( j == (fr2 - framestep) ) {
      xtr += dx2;
      ytr  = s - d10 + 2 * dy0Typical - correctedSideLenght;
      }
    else {
      if (j == framestep) float dxr = (xtr_plan - xtr) / segments_sum;
      dx = dxr * segments[(j - framestep)/ framestep];
      xtr += dx;
      ytr  = s - 64 + dy0Typical - dy * (j / 2) * fr2 / (fr2 - 2.0 * framestep);
      wr = correctedRotation - j * stepRotation;
      wl = wr;
      }
    xtl += dx2;
    ytl += dy0Typical;
    relaxFootR = (j == (fr2 - framestep) ? 1 : 0);
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalkStraight(2, j);
    }
  //testDrop();
  }

//Исполнить фазу 4
void walkStraightPhasa4() {
  //stepRotation = correctedRotation / (fr2 - 2.0 * framestep) * 2;
  ztl = -gaitHeight + stepHeight;
  
  for( j = 0; j < fr2; j += framestep ) {
    if( stepType == STEP_LAST ) {
      xtr_plan = xtl_plan = 0;
      if (j == (fr2 - framestep)){
        ztl = -gaitHeight;
        ytl = s + d10;
        }
      }
    else {
      xtr_plan = correctedStepLenght *( 0.5 - (fr1 + fr2) / (2.0 * fr1 + fr2));
      xtl_plan = correctedStepLenght / 2 + dx0Typical;
      dy = correctedSideLenght /(fr2- 2.0 * framestep) *framestep;
      }
    dx4 = (xtr_plan - xtr) / (fr2 - j) * framestep;
    if( j == 0 ) {
      xtl += dx4;
      ytl  = s + d10 + dy;
      }
    else if( j == (fr2 - framestep) ) {
      xtl += dx4;
      ytl  = s + d10 + 2 * dy - correctedSideLenght;
      }
    else {
      if (j == framestep) float dxl = (xtl_plan - xtl) / segments_sum;
      dx = dxl * segments[(j - framestep)/ framestep];
      xtl += dx;
      ytl  = s + 64 + dy0Typical - dy * j / 2.0;
      wr = j * stepRotation - correctedRotation;
      wl = wr;
      }
    xtr += dx4;
    ytr += dy0Typical;
    relaxFootL = (j == (fr2 - framestep) ? 1 : 0);
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalkStraight(4, j);
    }
  //testDrop();
  }


//Здесь отдельно алгоритм медленной ходьбы
//#include <roki3ASlowWalk.cpp>


float forwardDirection;

//Универсальный (для быстрой и медленной) ходьбы цикл
void walkCycle() {
  //Сохраняем стопу
  float xrOld = xr;
  float yrOld = yr;
  float xlOld = xl;
  float ylOld = yl;

    //Выполнить цикл быстрой ходьбы
    walkStraightInit();
    walkStraightPhasa1();
    walkStraightPhasa2();
    walkStraightPhasa3();
    walkStraightPhasa4();
    
  
  //Восстанавливаем стопу
  xr = xrOld;
  yr = yrOld;
  xl = xlOld;
  yl = ylOld;
  }

void stabilizeRotationByIMU(){
  //Корректировать направление
  sfQuaternionToEulerImu();
  rotation = -(svEulerYaw - forwardDirection);
  if( rotation > MATH_PI ) rotation -= 2 * MATH_PI;
  if( rotation < -MATH_PI ) rotation += 2 * MATH_PI;
  if( rotation > 0.7 ) rotation = 0.7;
  if( rotation < -0.7 ) rotation = -0.7;
  //rotation = 0;
}




//Факторы определяют степень зависимости поворота стоп от показаний IMU
int leftRightFactor; //Фактор миксования лево-право
int frontBackFactor; //Фактор миксования вперед-назад

void mixing() {
  //Начальные установки
  leftRightFactor = 100;
  frontBackFactor = 70;
  //В бесконечном цикле выполняем подмешивание
  while(1) {
    //По боковому крену
    leftFootSideAddonMix = -svImuGyroZ * leftRightFactor >> 10;
    rightFootSideAddonMix = svImuGyroZ * leftRightFactor >> 10;
    
    //По крену вперед-назад
    rightFootFrontAddonMix = leftFootFrontAddonMix = -svImuGyroX * frontBackFactor >> 10;
    
    //Ожидаем до следующего фрейма
    sfWaitNextFrame();
    }
  }
  
#define CYCLE_COUNT 20000 //Количество шагов. Общее количество шагов будет +3
#define STEP_LENGHT -10

void runTest() {
  //Стали в начальную позу
  walkStraightInitialPose();
  //Выполняем первый шаг
  stepType = STEP_FIRST;
  stepLength = STEP_LENGHT / 3.0;
  walkCycle();
  
  //Второй разгонный шаг
  stepType = STEP_OTHER;
  stepLength = STEP_LENGHT * 2.0 / 3.0;
  walkCycle();
  
  //Цикл одинаковых шагов
  stepLength = STEP_LENGHT;
  int i;
  for( i = 0; i < CYCLE_COUNT; i++ ){
    stabilizeRotationByIMU();
    walkCycle();
    }
  //Финальный шаг
  stepType = STEP_LAST;
  walkCycle();
  
  //Становимся в исходную позу
  walkStraightFinalPose();
  }



void main() {
  slowWalk = 0;
  setup();
  sideLength = -5;
  rotation = 0;
  relaxFootR = 0;
  relaxFootL = 0;

  //Процесс запускаем не сразу, а через некоторое время (100 фреймов - 1 сек)
  sfWaitFrame( 100 );
  //testDrop();
  //Фиксируем направление "вперед"
  sfQuaternionToEulerImu();
  forwardDirection = svEulerYaw;
  
  //Запускаем миксинг
  sfCreateTask( mixing, 20 );

  runTest();
  //Переходим к срипту "Напряжен"
  //sfStartSlot( sfSlotIndex("roki3TenseUp.cpp") );
  }

