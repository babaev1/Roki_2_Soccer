/*
  Проект "androSot"
  Автор
    Зубр ОВК
  Описание
    Скрипт движок ходьбы по методу Азера с дистанционным управлением
*/
#include <roki2met.h>

int slowWalk;

float stepLength; // = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
float sideLength; // = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
float rotation; // = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.
float stepLengthOrder;


float selfTIK2RAD;
float selfFrameDelay; // = self.glob.params['FRAME_DELAY']
//int   selfFramesPerCycle; // = self.glob.params['FRAMES_PER_CYCLE']
float selfMotionShiftCorrectionX; // = -self.glob.params['MOTION_SHIFT_TEST_X'] / 21
float selfMotionShiftCorrectionY; // = -self.glob.params['MOTION_SHIFT_TEST_Y'] / 21
float selfFirstStepYield; // = self.glob.first_step_yield
float selfCycleStepYield; // = self.glob.cycle_step_yield
float selfSideStepRightYield; // = self.glob.side_step_right_yield
float selfSideStepLeftYield; // = self.glob.side_step_left_yield
//        self.imu_drift_speed = math.radians(self.glob.params['IMU_DRIFT_IN_DEGREES_DURING_6_MIN_MEASUREMENT'])/360
int   selfFirstLegIsRightLeg; // = True
int   motion_to_right;
float   side_motion;
int   selfInitPoses; // = 400//self.simThreadCycleInMs

int   selfExitFlag; // = 0
int   selfFallingFlag; // = 0
int   selfNeckPan; // = 0
float rotationYieldRight;
float rotationYieldLeft;

int   framestep;

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
int timeStep;

float bodyTiltAtWalk;
float solyLandingSkew;

int stepNumber;

int flag;
int flag_event;

void setup() {
  flag_event = 0;
  timeStep = 1;
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
  
  gaitHeight = 180; // 180;  // Distance between Center of mass and floor in walk pose
  stepHeight = 32; //32.0; // elevation of sole over floor
  
  selfMotionShiftCorrectionX = -50.0 / 21.0;
  selfMotionShiftCorrectionY = 70.0 / 21.0;

  //self.LIMALPHA[3][1]=0

  ztr0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -223.1
  ztl0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -223.1
  
  zr = zl = -1;
  
  
  selfInitPoses = 10;
  
  selfTIK2RAD = 0.00058909;
  stepLength = 0.0;    // -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
  sideLength = 0.0;    // -20 - +20. Side step length to right (+) and to left (-)
  rotation = 0;        // -45 - +45 degrees Centigrade per step + CW, - CCW.
  selfFirstLegIsRightLeg = -1; // = True
  motion_to_right = 1;
  side_motion = 0.0;
  selfInitPoses = 20; //self.simThreadCycleInMs

  stepZtr = (ztr0 + gaitHeight) / selfInitPoses;
  stepZtl = (ztl0 + gaitHeight) / selfInitPoses;
  bodyTiltAtWalk = 0.02;
  solyLandingSkew = 0.00;
  
  if( slowWalk ) {
    fr1 = 50; //8;           // frame number for 1-st phase of gait ( two legs on floor)
    fr2 = 20; //12;          // frame number for 2-nd phase of gait ( one leg in air)
    amplitude = 110;    // mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
    }
  else {
    fr1 = 8;           // frame number for 1-st phase of gait ( two legs on floor)
    fr2 = 12;          // frame number for 2-nd phase of gait ( one leg in air)
    amplitude = 32;    // mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
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
  stepLengthOrder = 100;
  if (timeStep == 1){
    stepLengthOrder = 40;
    fps = 2; 
    bodyTiltAtWalk = -0.02; 
    gaitHeight = 150;
    stepHeight = 40;
    }
  }
  

//Вычисление углов серв с помощью обратной кинематики и отправка их в сервы
//Ожидание завершения перемещения
//Перемещение выполняется с помощью линейной интерполяции промежуточных значений
//fps определяет расчетную длительность шага, фактическая длительность на 1 меньше


int computeAlphaForWalk() {
  //Вычислить значения углов серв для правой ноги
  //if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  //else sfIkAngle( xtr, ytr, ztr, xr, yr, zr, -wr );
  flag = 0;
  sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  if( svIkOutPresent ) {
    flag = flag + 1;
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
    	sfPoseGroupLin( MASK_RIGHT_PELVIC, -svIkEncA5, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP, -svIkEncA7, fps );
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -svIkEncA8, fps );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE, -svIkEncA10, fps );
	//sfPoseGroupLin( MASK_RIGHT_CLAVICLE, (1400 - xtl * 30), fps );
      }
    else {
      sfPoseGroupLin( MASK_LEFT_PELVIC, svIkEncA5, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP, -svIkEncA7, fps );
    	sfPoseGroupLin( MASK_LEFT_KNEE, -svIkEncA8, fps );
    	sfPoseGroupLin( MASK_LEFT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroupLin( MASK_LEFT_FOOT_SIDE, -svIkEncA10, fps );
	//sfPoseGroupLin( MASK_LEFT_CLAVICLE, (1400 - xtr * 30), fps );
      }
    }
  //Вычислить значение углов серв для левой ноги
  //if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  //else sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, -wl );
  sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  if( svIkOutPresent ) {
    flag = flag + 1;
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
    	sfPoseGroupLin( MASK_LEFT_PELVIC, -svIkEncA5, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroupLin( MASK_LEFT_HIP, -svIkEncA7, fps );
    	sfPoseGroupLin( MASK_LEFT_KNEE, -svIkEncA8, fps );
    	sfPoseGroupLin( MASK_LEFT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroupLin( MASK_LEFT_FOOT_SIDE, -svIkEncA10, fps );
	//sfPoseGroupLin( MASK_LEFT_CLAVICLE, (1400 - xtr * 30), fps );
      }
    else {
      sfPoseGroupLin( MASK_RIGHT_PELVIC, svIkEncA5, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroupLin( MASK_RIGHT_HIP, -svIkEncA7, fps );
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -svIkEncA8, fps );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE, -svIkEncA10, fps );
	//sfPoseGroupLin( MASK_RIGHT_CLAVICLE, (1400 - xtl * 30), fps );
      }
    }
  //Ожидать когда движение завершится
  sfWaitFrame( timeStep );
  if (flag < 2) flag_event = flag_event + 1;
  return 1;
  }





//Вычисление углов серв с помощью обратной кинематики и отправка их в сервы
//Ожидание завершения перемещения
//Перемещение выполняется с помощью синусоидальной интерполяции промежуточных значений
//frameCount - количество фреймов интерполяции и фактическая длительность
void computeAlphaForWalkFine( int frameCount ) {
  //Вычислить значения углов серв для правой ноги
  //if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  //else sfIkAngle( xtr, ytr, ztr, xr, yr, zr, -wr );
  sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  //Ни одного решения не найдено
  if( svIkOutPresent ) {
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
    	sfPoseGroupLin( MASK_RIGHT_PELVIC, -svIkEncA5, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_HIP_SIDE, -svIkEncA6, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_HIP, -svIkEncA7, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -svIkEncA8, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT, svIkEncA9, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE, -svIkEncA10, frameCount );
      }
    else {
	sfPoseGroupLin( MASK_LEFT_PELVIC, svIkEncA5, frameCount );
    	sfPoseGroupLin( MASK_LEFT_HIP_SIDE, -svIkEncA6, frameCount );
    	sfPoseGroupLin( MASK_LEFT_HIP, -svIkEncA7, frameCount );
    	sfPoseGroupLin( MASK_LEFT_KNEE, -svIkEncA8, frameCount );
    	sfPoseGroupLin( MASK_LEFT_FOOT_FRONT, svIkEncA9, frameCount );
    	sfPoseGroupLin( MASK_LEFT_FOOT_SIDE, -svIkEncA10, frameCount );
      }
    }
  //Вычислить значение углов серв для левой ноги
  //if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  //else sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, -wl );
  sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  if( svIkOutPresent ) {
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
    	sfPoseGroupLin( MASK_LEFT_PELVIC, -svIkEncA5, frameCount );
    	sfPoseGroupLin( MASK_LEFT_HIP_SIDE, -svIkEncA6, frameCount );
    	sfPoseGroupLin( MASK_LEFT_HIP, -svIkEncA7, frameCount );
    	sfPoseGroupLin( MASK_LEFT_KNEE, -svIkEncA8, frameCount );
    	sfPoseGroupLin( MASK_LEFT_FOOT_FRONT, svIkEncA9, frameCount );
    	sfPoseGroupLin( MASK_LEFT_FOOT_SIDE, -svIkEncA10, frameCount );
      }
    else {
      sfPoseGroupLin( MASK_RIGHT_PELVIC, svIkEncA5, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_HIP_SIDE, -svIkEncA6, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_HIP, -svIkEncA7, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -svIkEncA8, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT, svIkEncA9, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE, -svIkEncA10, frameCount );
      }
    }
  //Ожидать когда движение завершится
  sfPoseGroupLin( MASK_RIGHT_ELBOW | MASK_LEFT_ELBOW, 4540, frameCount );
  sfPoseGroupLin( MASK_RIGHT_CLAVICLE | MASK_LEFT_CLAVICLE, 1400, frameCount );
  sfPoseGroupLin( MASK_RIGHT_ELBOW_SIDE | MASK_LEFT_ELBOW_SIDE, 700, frameCount );
  sfWaitFrame( frameCount );
  }




//Проверить падение
void testDrop() {
  if( sfAbs(svImuAccX) > 50000 || sfAbs(svImuAccZ) > 50000 ) {
    //Произошло падение
    //Все расслабить
    //sfFreeGroup( MASK_ALL );
    //Подождать 1 сек
    sfWaitFrame(100);
    //Перейти к вставанию
    //sfStartSlot( sfSlotIndex("androSotStandUp.cpp") );
    }
  }

int j;

//Установить начальную позу
//Если робот был не исходной позе (ровный на высоте ztr0), то робота дергает
void walkInitialPose() {
  xtr = xtl = 0;
  for( j = 0; j < selfInitPoses; j++ ) {
    //Задать положение стоп для двух ног
    ztr = ztr0 - j * stepZtr;
    ztl = ztl0 - j * stepZtl;
    ytr = -d10 - j * stepYtr;
    ytl =  d10 - j * stepYtl;
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    }
  }


//Установить начальную позу
//Алгоритм с использованием синусоидального движения
//Мы вычисляем конечную позу и за заданное количество фреймов робот плавно ее достигает
//вне зависимости от исходного состояния
void walkInitialPoseFine() {
  xtr = xtl = 0;
  ztr = ztl = -gaitHeight;
  ytr = -d10 - amplitude / 2.0;
  ytl =  d10 - amplitude / 2.0;
  computeAlphaForWalkFine( 40 );
  }

//Установить исходную позу (ровный на высоте ztr0)
void walkFinalPose() {
  for( j = 0; j < selfInitPoses; j++ ) {
    //Задать положение стоп двух ног
    ztr = -gaitHeight + (j+1) * stepZtr;
    ztl = -gaitHeight + (j+1) * stepZtr;
    ytr = -d10 + (selfInitPoses - (j+1)) * stepYtr;
    ytl =  d10 + (selfInitPoses - (j+1)) * stepYtl;
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    }
  }



//Установить исходную позу (ровный на высоте ztr0)
//Алгоритм с использованием синусоидального движения
//Мы вычисляем конечную позу и за заданное количество фреймов робот плавно ее достигает
//вне зависимости от исходного состояния
void walkFinalPoseFine() {
  xtr = xtl = 0;
  ztr = ztl = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1);
  ytr = -d10;
  ytl =  d10;
  computeAlphaForWalkFine( 40 );
  }

float xtl0;
float xtr0;
float dx0Typical;
float dy0Typical;
float correctedRotation;
float s;
float fase_offset;

//Настроить следующий цикл хотьбы
void walkInit() {
  //Корректируем длину шага и движение в сторону
  correctedStepLenght = stepLength + selfMotionShiftCorrectionX;
  correctedStepLenghtHalf = correctedStepLenght / 2;
  correctedSideLenght = sideLength - selfMotionShiftCorrectionY;
  correctedSideLenghtHalf = correctedSideLenght / 2;
  
  //Корректируем поворот
  correctedRotation = -rotation *0.25 * 0.23 / ( rotation <= 0 ? rotationYieldRight : rotationYieldLeft);
  //correctedRotation = -rotation * 0.23 / ( rotation <= 0 ? rotationYieldRight : rotationYieldLeft);
  xtl0 = correctedStepLenght * (1 - (fr1 + fr2 + 2.0 * framestep) / (2 * fr1 + fr2 + 2.0 * framestep)) * 1.5;
  xtr0 = correctedStepLenght * (0.5 - (fr1 + fr2 + 2.0 * framestep) / (2 * fr1 + fr2 + 2.0 * framestep));
  dx0Typical = correctedStepLenght / (2 * fr1 + fr2 + 2.0 * framestep) * framestep;
  dy0Typical = correctedSideLenght / (2.0 * fr2) * framestep;
  
  //Некоторые константные настройки
  xr = xl = bodyTiltAtWalk;
  yl =  solyLandingSkew;
  yr = -solyLandingSkew;
  fase_offset = 0.7;
  }


//Исполнить фазу 1
void walkPhasa1() {
  ztl = ztr = -gaitHeight;
  for( j = 0; j < fr1; j += framestep ) {
    //s = amplitude / 2 * sfMathCos( alpha01 * j / 2 );
    s = (amplitude / 2 + correctedSideLenghtHalf) * sfMathCos(alpha01 * (j / 2 + fase_offset * framestep));
    ytr = s - d10 + correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    if( stepType == STEP_FIRST ) continue;
    xtl = xtl0 - dx0Typical - dx0Typical * j / framestep;
    xtr = xtr0 - dx0Typical - dx0Typical * j / framestep;
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    //testDrop();
    }
  }

//Исполнить фазу 3
void walkPhasa3() {
  ztl = ztr = -gaitHeight;
  for( j = 0; j < fr1; j += framestep ) {
    //s = amplitude / 2 * sfMathCos( alpha01 * (j + fr1) / 2 );
    s = (amplitude / 2 + correctedSideLenghtHalf) * sfMathCos(alpha01 * ((j -fr2) / 2 + fase_offset * framestep));
    ytr = s - d10 - correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    xtl -= dx0Typical;
    xtr -= dx0Typical;
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    //testDrop();
    }
  }
  
float dx,dy;
float stepRotation;

//Исполнить фазу 2
void walkPhasa2() {
  ztr = -gaitHeight + stepHeight;
  //dx0 == dx0Typical при любых обстоятельствах
  //dy0 == dy0Typical при любых обстоятельствах
  
  //dy = self.sideLength/self.fr2*framestep во всех случаях
  //единственное использование dy
  //self.ytr = S - 64 + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
  //Поэтому выносим выражение dy*self.fr2/(self.fr2- 2 * framestep) за цикл
  //Подставляем dy -> self.sideLength/self.fr2*framestep*self.fr2/(self.fr2- 2 * framestep)
  //Получается новое значение dy = self.sideLength/self.fr2*framestep*self.fr2/(self.fr2- 2 * framestep) 
  //                             = self.sideLength*framestep/(self.fr2 - 2 * framestep)
  //которое мы можем использовать в 
  //self.ytr = S - 64 + dy0 - dy*((iii - self.fr1)/2)
  dy = correctedSideLenght * framestep / (fr2 - 2.0 * framestep);
  
  //Из выражения self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
  //выносим за цикл то, что не изменяется rotation/(self.fr2- 2 * framestep)*2
  stepRotation = correctedRotation / (fr2 - 2.0 * framestep) * 2;
  
  if( stepType == STEP_FIRST )
    dx = correctedStepLenght / (fr2 - 2.0 * framestep) * framestep / 2;
  else 
    dx = correctedStepLenght / (fr2 - 2.0 * framestep) * framestep;
    
  for( j = 0; j < fr2; j += framestep ) {
    if( j == 0 ) {
      xtr -= dx0Typical;
      ytr  = s - d10 + dy0Typical;
      }
    else if( j == (fr2 - framestep) ) {
      xtr -= dx0Typical;
      ytr  = s - d10 + 2 * dy0Typical - correctedSideLenght;
      }
    else {
      xtr += dx;
      ytr  = s - 64 + dy0Typical - dy * (j / 2);
      wr = correctedRotation - j * stepRotation;
      wl = wr;
      }
    xtl -= dx0Typical;
    ytl += dy0Typical;

    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    //testDrop();
    }
  }

float dx0;

//Исполнить фазу 4
void walkPhasa4() {
  stepRotation = correctedRotation / (fr2 - 2.0 * framestep) * 2;
  ztl = -gaitHeight + stepHeight;
  if( stepType == STEP_LAST ) {
    dx0 = dx0Typical * 4 / fr2;
    dx = (correctedStepLenght * (fr1 + fr2) / (4 * fr1) + 2.0 * dx0) / (fr2 - 2.0 * framestep) * framestep / 1.23076941;
    }
  else {
    dx = correctedStepLenght / (fr2 - 2.0 * framestep) * framestep;
    dx0 = dx0Typical;
    }
  for( j = 0; j < fr2; j += framestep ) {
    if( stepType == STEP_LAST && j == (fr2 - framestep) ) {
      ztl = -gaitHeight;
      ytl = s + d10;
      }
    if( j == 0 ) {
      xtl -= dx0;
      ytl  = s + 64 + correctedSideLenghtHalf;
      }
    else if( j == (fr2 - framestep) ) {
      xtl -= dx0;
      ytl  = s + d10 + correctedSideLenghtHalf;
      }
    else {
      xtl += dx;
      ytl  = s + 64 + correctedSideLenghtHalf;
      wr = j * stepRotation - correctedRotation;
      wl = wr;
      }
    xtr -= dx0;
    ytr += dy0Typical;
    
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    //testDrop();
    }
  }

//Здесь отдельно алгоритм медленной ходьбы
//#include <roki3ASlowWalk.cpp>


float forwardDirection;

//Универсальный (для быстрой и медленной) ходьбы цикл
// if half == 1 then just half of cycle executed
// if half == 0 then full cycle executed
void walkCycle(int half) {
  //Сохраняем стопу
  float xrOld = xr;
  float yrOld = yr;
  float xlOld = xl;
  float ylOld = yl;

  //if( slowWalk ) {
    //Выполнить цикл медленной ходьбы
    //slowWalkInit();
    //slowWalkPhasa1();
    //slowWalkPhasa2();
    //if (half == 0){
      //slowWalkPhasa3();
      //slowWalkPhasa4();
    //  }
    //}
  //else {
    //Выполнить цикл быстрой ходьбы
    walkInit();
    walkPhasa1();
    walkPhasa2();
    if (half == 0){
      walkPhasa3();
      walkPhasa4();
      }
    //}
  
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
  frontBackFactor = 100;
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




#define CYCLE_COUNT 50 //Количество шагов. Общее количество шагов будет +3
//#define STEP_LENGHT 70

void runTest() {
  //Стали в начальную позу
  walkInitialPoseFine();
  //Выполняем первый шаг
  stepType = STEP_FIRST;
  stepLength = stepLengthOrder/ 3.0;
  walkCycle(0);
  
  //Второй разгонный шаг
  stepType = STEP_OTHER;
  stepLength = stepLengthOrder * 2.0 / 3.0;
  walkCycle(0);
  
  //Цикл одинаковых шагов
  stepLength = stepLengthOrder;
  int i;
  for( i = 0; i < CYCLE_COUNT; i++ ){
    stabilizeRotationByIMU();
    walkCycle(0);
    }
  //Финальный шаг
  stepType = STEP_LAST;
  walkCycle(0);
  
  //Становимся в исходную позу
  walkFinalPoseFine();
  }


//Преобразование положения джойстика в длину шага
//Положительные значения находятся в диапазоне от 0 до 100, если принять во внимание
// что максимальная длина шага 50мм, то преобразование будет joystick * 50 / 100
int joystickToStepLength() {
  //Здесь 50 - максимальная длина шага, если максимальная длина другая - то поменять здесь число
  return -svRemoteRightJoystickY * 50 / 100;
  }

//Преобразование джойстика в длину в сторону
int joystickToSideLength() {
  //Здесь 20 - максимальная величина в сторону
  side_motion = - svRemoteRightJoystickX * 20.0 / 100.0;
  motion_to_right = (side_motion <= 0 ? 1:0);
  return sfAbs (side_motion);
  }

//Преобразование джойстика в поворот
int joystickToRotation() {
  //Здесь 260 - 0.260 радиан максимальный угол поворота
  return -svRemoteLeftJoystickX * 260 / 100;
  }

//Цикл хотьбы. Длина шага определяется правым джойстиком svRemoteRightJoystickY
void walkLoop() {
  int i;
  //Выполняем первый шаг
  stepType = STEP_FIRST;
  stepLength = joystickToStepLength() / 3.0;
  sideLength = joystickToSideLength();
  selfFirstLegIsRightLeg = motion_to_right;
  forwardDirection += joystickToRotation() / 1000.0;
  stabilizeRotationByIMU();
  walkCycle(0);
  stepNumber -= 1;
  
  //Проверим не бросили ли джойстик
  if( sfAbs( svRemoteRightJoystickX ) < 10 
   && sfAbs( svRemoteRightJoystickY ) < 10 
   && sfAbs( svRemoteLeftJoystickX ) < 10 
   && !(svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1))
   && stepNumber < 1)
    //Джойстик бросили, выходим
    return;
  
  
  //Второй разгонный шаг
  stepType = STEP_OTHER;
  stepLength = joystickToStepLength() * 2.0 / 3.0;
  sideLength = joystickToSideLength();
  selfFirstLegIsRightLeg = motion_to_right;
  forwardDirection += joystickToRotation() / 1000.0;
  stabilizeRotationByIMU();
  walkCycle(0);
  stepNumber -= 1;
  
  //Проверим не бросили ли джойстик
  if( sfAbs( svRemoteRightJoystickX ) < 10 
   && sfAbs( svRemoteRightJoystickY ) < 10 
   && sfAbs( svRemoteLeftJoystickX ) < 10 
   && !(svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1))
   && stepNumber < 1)
    //Джойстик бросили, выходим
    return;
  
  //Цикл одинаковых шагов пока не бросят джойстик
  while( sfAbs( svRemoteRightJoystickX ) >= 10 || sfAbs( svRemoteRightJoystickY ) >= 10 || sfAbs( svRemoteLeftJoystickX ) >= 10) {
    //Установить длину шага в соответствии с джойстиком
    stepLength = joystickToStepLength();
    sideLength = joystickToSideLength();
    forwardDirection += joystickToRotation() / 1000.0;
    stabilizeRotationByIMU();
    if (selfFirstLegIsRightLeg != motion_to_right) walkCycle(1); // if direction left/right is changed then half cycle is added
    selfFirstLegIsRightLeg = motion_to_right;
    walkCycle(0);
    stepNumber -= 1;
    if ((svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1)) || stepNumber > 0){
      if (svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1)) stepNumber = -svRemoteLeftJoystickY;
      for( i = 0; i < stepNumber ; i++ ){
        stabilizeRotationByIMU();
        walkCycle(0);
        }
      }
    }
  }

void kick(int kickByRight, int small){
  float gaitHeightKick = 200;
  float stepHeightKick = 40;
  float stepLengthKick = 50;
  float kick_size = 100;
  float bodyTiltAtKick = 0.04;
  int pose_taking_cycles = 20;
  //Сохраняем стопу
  float xrOld = xr;
  float yrOld = yr;
  float xlOld = xl;
  float ylOld = yl;
  if (small == 1) kick_size = -10;
  selfFirstLegIsRightLeg = kickByRight;
  dx0Typical = stepLengthKick / (2 * fr1 + fr2 + 2.0 * framestep) * framestep;
  xr = xl = bodyTiltAtKick;
  yl =  solyLandingSkew;
  yr = -solyLandingSkew;
  s = (amplitude / 2 ) * sfMathCos(alpha01 * (fr1 / 2 + 0.5 * framestep));
  xtr = xtl = 0;

  // initial pose
  ztr = ztl = -gaitHeightKick;
  //ytr = -d10 - amplitude*3 ;
  //ytl =  d10 - amplitude*3 ;
  //computeAlphaForWalkFine( 40 );
  for( j = 0; j < pose_taking_cycles; j += 1 ){
    ztr = ztr0 + j * (-ztr0 - gaitHeightKick) / pose_taking_cycles;
    ztl = ztr;
    ytr = -d10 - j * amplitude * 3 /pose_taking_cycles;
    ytl =  ytr + 2 * d10;
    computeAlphaForWalk();
    }
  
  ztr = ztl = -gaitHeightKick;
  ytr = s - d10;
  ytl = s + d10;
  
  for( j = fr1; j < fr1 + fr2; j += framestep ){
    ztr = -gaitHeightKick + stepHeightKick;
    dx = stepLengthKick / fr2;
    dx0 = stepLengthKick/( 2 * fr1 + fr2 + 4 ) * framestep;
    if (j == fr1 || j == fr1 + fr2 - 2){
      xtr -= dx0;
      ytr = s - 64;
      }
    else
      xtr += dx * fr2 / (fr2 - 2 * framestep);
    if (j == fr1 + fr2 - 10)
      xtr += kick_size;
      xtl -= kick_size;
    if (j == fr1 + fr2 - 4)
      xtr -= kick_size;
      xtl += kick_size;
    xtl -= dx0;
    computeAlphaForWalk();
    }
  for( j = fr1 + fr2; j < 2 * fr1 + fr2; j += framestep ){
    s = (amplitude / 2 ) * sfMathCos(alpha01 * (j - fr2 + framestep) / 2);
    ytr = s - d10;
    ytl = s + d10;
    ztl = -gaitHeightKick;
    ztr = -gaitHeightKick;
    dx0 = dx0Typical;
    xtl -= dx0;
    xtr -= dx0;
    computeAlphaForWalk();
    }
  for( j = 2 * fr1 + fr2; j < 2 * (fr1 + fr2); j += framestep ){
    ztl = -gaitHeightKick + stepHeightKick;
    dx0 = dx0Typical * 4 / fr2;
    dx = (stepLengthKick * (fr1 + fr2) / (4 * fr1) + 2 * dx0) / (fr2 - 2 * framestep) * framestep;
    if (j == (2 * fr1 + 2 * fr2 - framestep)){
      ztl = -gaitHeightKick;
      ytl = s + d10;
      }
    if (j == (2 * fr1 + fr2 ) || j == (2 * (fr1 + fr2) - framestep)){
      xtl -= dx0;
      ytl = s + 64;
      }
    else
      xtl += dx;
    xtr -= dx0;
    computeAlphaForWalk();
    }

  // walk_Final_Pose_After_Kick
  pose_taking_cycles = 2;
  int pose_hight_after_kick = ztr0;
  for( j = 0; j < pose_taking_cycles; j += 1 ){
    ztr = -gaitHeightKick + (j + 1 )*(pose_hight_after_kick + gaitHeightKick) / pose_taking_cycles;
    ztl = ztr;
    ytr = -d10 - (pose_taking_cycles - (j + 1)) * amplitude / 2 / pose_taking_cycles;
    ytl = d10 - (pose_taking_cycles - (j + 1)) * amplitude / 2 / pose_taking_cycles;
    computeAlphaForWalk();
    }
  //Восстанавливаем стопу
  xr = xrOld;
  yr = yrOld;
  xl = xlOld;
  yl = ylOld;
}

void turn(int direction){
  int frameCount = 4;
  if (direction == 1){
    sfPoseGroup( MASK_RIGHT_PELVIC, -2000, frameCount );
    sfPoseGroup( MASK_LEFT_PELVIC, 2000, frameCount );
    }
  if (direction == -1){
    sfPoseGroup( MASK_RIGHT_PELVIC, 2000, frameCount );
    sfPoseGroup( MASK_LEFT_PELVIC, -2000, frameCount );
    }
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 2000, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 2000, frameCount );
  sfWaitFrame( frameCount );
  sfPoseGroup( MASK_RIGHT_PELVIC, 0, frameCount );
  sfPoseGroup( MASK_LEFT_PELVIC, 0, frameCount );
  sfPoseGroup( MASK_RIGHT_FOOT_SIDE, 0, frameCount );
  sfPoseGroup( MASK_LEFT_FOOT_SIDE, 0, frameCount );
  sfWaitFrame( frameCount );
  sfWaitFrame( 6 );
}

void movingRemoteControl(){

  while(1) {
    //Ожидаем пока не будет нажат джойстик вперед
    while( sfAbs( svRemoteRightJoystickX ) < 10  
        && sfAbs( svRemoteRightJoystickY ) < 10 
        && sfAbs( svRemoteLeftJoystickX ) < 10 
        && !(svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1))){ // одновременное нажатие кнопки START и левый джойстик вверх
      //if (svRemoteButton == ARB_GETUP_STOMACH || svRemoteButton == ARB_GETUP_FACEUP) sfStartSlot( sfSlotIndex("androSotStandUp.cpp") );
      if (svRemoteButton == ARB_KICK_RIGHT || svRemoteButton == ARB_R2) kick(1,0); // удар по мячу правой ногой
      if (svRemoteButton == ARB_KICK_LEFT || svRemoteButton == ARB_L2) kick(0,0); // удар по мячу левой ногой
      if (svRemoteButton == ARB_A) turn(1); // поворот в право
      if (svRemoteButton == ARB_Y) turn(-1); // поворот в право
      sfWaitNextFrame();
    	}
    
    //Джойстик нажали. Переходим к циклу хотьбы
    sideLength = joystickToSideLength();
    selfFirstLegIsRightLeg = motion_to_right;
    if (svRemoteLeftJoystickY < 0 && ((svRemoteButton / 2048) % 2 == 1)) stepNumber = -svRemoteLeftJoystickY;
    //Стали в начальную позу
    walkInitialPoseFine();
    walkLoop();
  
    //Финальный шаг
    stepType = STEP_LAST;
    walkCycle(0);
    //Становимся в исходную позу
    walkFinalPoseFine();
    }
  
  //Становимся в исходную позу
  walkFinalPoseFine();
}

void main() {
  slowWalk = 0;
  setup();
  sideLength = 0;
  rotation = 0;

  //Процесс запускаем не сразу, а через некоторое время (100 фреймов - 1 сек)
  sfWaitFrame( 100 );
  
  //Фиксируем направление "вперед"
  sfQuaternionToEulerImu();
  forwardDirection = svEulerYaw;
  
  //Запускаем миксинг
  sfCreateTask( mixing, 20 );

  //movingRemoteControl(); // движение с пультом ДУ
  runTest(); // простой тест ходьбы прямо
  //kick(1,0); // удар по мячу
  
  //Переходим к срипту "Напряжен"
  //sfStartSlot( sfSlotIndex("androSotTenseUp.cpp") );
  }

