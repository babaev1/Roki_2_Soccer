/*
  Проект "Roki-2 FIRA"
  Автор
    Зубр ОВК
  Описание
    Скрипт движок ходьбы по методу Азера для Спринт
*/
#include <roki2met.h>

int restart_flag;
int timeStep;
int orderFromHead; // 0 - no order, 1 - straight forward, 2 - to left, 3- to right, 4 - reverse back.
int pitStop;
int startStop;
int cycle_number;
int hipTilt;
int stepLengthOrder;
int fps;
int fr1;
int fr2;
int gaitHeight;
int stepHeight;

float bodyTiltAtWalk;
float solyLandingSkew;
float ugol_torsa;

// Do not change order of move above lines in order to preserve global variable IDs 
// used for communucation with head. 

int slowWalk;
float stepLength;
float sideLength; // = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
float rotation; // = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.

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
int flag;
int flag_event;

float dobavka_x_ot_torsa;
float tors_angle;

void setup() {
  orderFromHead = 0;
  flag_event = 0;
  cycle_number = 10;
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
  
  
  
  selfMotionShiftCorrectionX = 0.0;
  selfMotionShiftCorrectionY = 0.0;

  //self.LIMALPHA[3][1]=0

  ztr0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -223.1
  ztl0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -223.1
  
  zr = zl = -1;
  
  
  selfInitPoses = 10;
  

  stepLength = 0.0;    // -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
  sideLength = 0.0;    // -20 - +20. Side step length to right (+) and to left (-)
  rotation = 0;        // -45 - +45 degrees Centigrade per step + CW, - CCW.
  selfFirstLegIsRightLeg = 1; // = True
  motion_to_right = 1;
  side_motion = 0.0;
  selfInitPoses = 20; //self.simThreadCycleInMs

  stepZtr = (ztr0 + gaitHeight) / selfInitPoses;
  stepZtl = (ztl0 + gaitHeight) / selfInitPoses;
  
  solyLandingSkew = 0.00;
  
  tors_angle = 0;     // current tors angle value

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

  timeStep = 2;
  
  stepLengthOrder = 50;
  ugol_torsa = 0.3;  	// tors maximum turning angle in radians
  bodyTiltAtWalk = 0.055;
  hipTilt = 0;
  gaitHeight = 190; // 180;  // Distance between Center of mass and floor in walk pose
  stepHeight = 40; //32.0; // elevation of sole over floor
  fps = 4;
  fr1 = 5;           // frame number for 1-st phase of gait ( two legs on floor)
  fr2 = 7;          // frame number for 2-nd phase of gait ( one leg in air)
  amplitude = 32;    // mm side amplitude (maximum distance between most right and most left position of Center of Mass) 62 * 2
  
  if (timeStep == 1){
    stepLengthOrder = 40;
    ugol_torsa = 0.65;
    bodyTiltAtWalk = -0.02; 
    hipTilt = 80;
    gaitHeight = 135;
    stepHeight = 35;
    fps = 2;
    fr1 = 4;           // frame number for 1-st phase of gait ( two legs on floor)
    fr2 = 9;          // frame number for 2-nd phase of gait ( one leg in air)
    amplitude = 40;    // mm side amplitude (maximum distance between most right and most left position of Center of Mass) 62 * 2
    }
  }
  

//Вычисление углов серв с помощью обратной кинематики и отправка их в сервы
//Ожидание завершения перемещения
//Перемещение выполняется с помощью линейной интерполяции промежуточных значений
//fps определяет расчетную длительность шага, фактическая длительность на 1 меньше

int torsoAdd;
float forwardDirection;
float correctedRotation;

void sitToStart(int frameCount){
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

void stabilizeRotationByIMU(){
  //Корректировать направление
  sfQuaternionToEulerImu();
  rotation = (forwardDirection - svEulerYaw) * 1.1;
  if( rotation > MATH_PI ) rotation -= 2 * MATH_PI;
  if( rotation < -MATH_PI ) rotation += 2 * MATH_PI;
  if( rotation > 0.3 ) rotation = 0.3;
  if( rotation < -0.3 ) rotation = -0.3;
  correctedRotation = rotation * 0.25 * 0.23 / (rotation <= 0 ? rotationYieldRight : rotationYieldLeft);
  if (orderFromHead == 1)correctedRotation = 0;
  else if (orderFromHead == 2) correctedRotation = 0.3;
  else if (orderFromHead == 3) correctedRotation = -0.3;
  //rotation = 0;
}

int computeAlphaForWalk() {
  //Вычислить значения углов серв для правой ноги
  //if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  //else sfIkAngle( xtr, ytr, ztr, xr, yr, zr, -wr );
  flag = 0;
  torsoAdd = tors_angle * ENC_PER_RADIAN;
  stabilizeRotationByIMU();
  if (correctedRotation > 0) {
    xtr *= 1.3;
    xtl *= 0.7;
    }
  if (correctedRotation < 0) {
    xtr *= 0.7;
    xtl *= 1.3;
    }
  sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
  if( svIkOutPresent ) {
    flag = flag + 1;
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
    	sfPoseGroup( MASK_RIGHT_PELVIC, -(svIkEncA5 - torsoAdd), fps );
    	sfPoseGroup( MASK_RIGHT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroup( MASK_RIGHT_HIP, -svIkEncA7 + hipTilt, fps );
    	sfPoseGroup( MASK_RIGHT_KNEE, -svIkEncA8 / 2, fps );
          sfPoseGroup( MASK_RIGHT_KNEE_BOT, -svIkEncA8 / 2, fps );
    	sfPoseGroup( MASK_RIGHT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroup( MASK_RIGHT_FOOT_SIDE, -svIkEncA10, fps );
        	sfPoseGroup( MASK_TORSO_ROTATE, -torsoAdd, fps);
	sfPoseGroup( MASK_RIGHT_CLAVICLE, (1400 - xtl * 30), fps );
      }
    else {
      sfPoseGroupLin( MASK_LEFT_PELVIC, (svIkEncA5 - tors_angle), fps );
    	sfPoseGroup( MASK_LEFT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroup( MASK_LEFT_HIP, -svIkEncA7 + hipTilt, fps );
    	sfPoseGroup( MASK_LEFT_KNEE, -svIkEncA8 / 2, fps );
    	sfPoseGroup( MASK_LEFT_KNEE_BOT, -svIkEncA8 / 2, fps );
    	sfPoseGroup( MASK_LEFT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroup( MASK_LEFT_FOOT_SIDE, -svIkEncA10, fps );
        	sfPoseGroup(MASK_TORSO_ROTATE, -torsoAdd, fps);
	sfPoseGroup( MASK_LEFT_CLAVICLE, (1400 - xtr * 30), fps );
      }
    }
  //Вычислить значение углов серв для левой ноги
  //if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  //else sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, -wl );
  sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
  if (correctedRotation > 0) {
    xtr /= 1.3;
    xtl /= 0.7;
    }
  if (correctedRotation < 0) {
    xtr /= 0.7;
    xtl /= 1.3;
    }
  if( svIkOutPresent ) {
    flag = flag + 1;
    //Записать новые значения в сервы
    if (selfFirstLegIsRightLeg == 1){
    	sfPoseGroup( MASK_LEFT_PELVIC, -(svIkEncA5 + torsoAdd), fps );
    	sfPoseGroup( MASK_LEFT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroup( MASK_LEFT_HIP, -svIkEncA7 + hipTilt, fps );
    	sfPoseGroup( MASK_LEFT_KNEE, -svIkEncA8 / 2, fps );
      	sfPoseGroup( MASK_LEFT_KNEE_BOT, -svIkEncA8 / 2, fps );
    	sfPoseGroup( MASK_LEFT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroup( MASK_LEFT_FOOT_SIDE, -svIkEncA10, fps );
	sfPoseGroup( MASK_LEFT_CLAVICLE, (1400 - xtr * 30), fps );
      }
    else {
      sfPoseGroup( MASK_RIGHT_PELVIC, (svIkEncA5 - tors_angle), fps );
    	sfPoseGroup( MASK_RIGHT_HIP_SIDE, -svIkEncA6, fps );
    	sfPoseGroup( MASK_RIGHT_HIP, -svIkEncA7 + hipTilt, fps );
    	sfPoseGroup( MASK_RIGHT_KNEE, -svIkEncA8 / 2, fps );
      	sfPoseGroup( MASK_RIGHT_KNEE_BOT, -svIkEncA8 / 2, fps );
    	sfPoseGroup( MASK_RIGHT_FOOT_FRONT, svIkEncA9, fps );
    	sfPoseGroup( MASK_RIGHT_FOOT_SIDE, -svIkEncA10, fps );
	sfPoseGroup( MASK_RIGHT_CLAVICLE, (1400 - xtl * 30), fps );
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
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -svIkEncA8 / 2, frameCount );
      	sfPoseGroupLin( MASK_RIGHT_KNEE_BOT, -svIkEncA8 / 2, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_FRONT, svIkEncA9, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_FOOT_SIDE, -svIkEncA10, frameCount );
      }
    else {
	sfPoseGroupLin( MASK_LEFT_PELVIC, svIkEncA5, frameCount );
    	sfPoseGroupLin( MASK_LEFT_HIP_SIDE, -svIkEncA6, frameCount );
    	sfPoseGroupLin( MASK_LEFT_HIP, -svIkEncA7, frameCount );
    	sfPoseGroupLin( MASK_LEFT_KNEE, -svIkEncA8 / 2, frameCount );
      	sfPoseGroupLin( MASK_LEFT_KNEE_BOT, -svIkEncA8 / 2, frameCount );
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
    	sfPoseGroupLin( MASK_LEFT_KNEE, -svIkEncA8 / 2, frameCount );
      	sfPoseGroupLin( MASK_LEFT_KNEE_BOT, -svIkEncA8 / 2, frameCount );
    	sfPoseGroupLin( MASK_LEFT_FOOT_FRONT, svIkEncA9, frameCount );
    	sfPoseGroupLin( MASK_LEFT_FOOT_SIDE, -svIkEncA10, frameCount );
      }
    else {
      sfPoseGroupLin( MASK_RIGHT_PELVIC, svIkEncA5, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_HIP_SIDE, -svIkEncA6, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_HIP, -svIkEncA7, frameCount );
    	sfPoseGroupLin( MASK_RIGHT_KNEE, -svIkEncA8 / 2, frameCount );
      	sfPoseGroupLin( MASK_RIGHT_KNEE_BOT, -svIkEncA8 / 2, frameCount );
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

float s;
float fase_offset;
float dx, dy, dx1, dx2, dx4;
float stepRotation;
float xtl_plan, xtr_plan;

//Настроить следующий цикл хотьбы
void walkInit() {
  //Корректируем длину шага и движение в сторону
  correctedStepLenght = stepLength + selfMotionShiftCorrectionX;
  correctedStepLenghtHalf = correctedStepLenght / 2;
  correctedSideLenght = sideLength - selfMotionShiftCorrectionY;
  correctedSideLenghtHalf = correctedSideLenght / 2;
  
  //Корректируем поворот
  
  //Из выражения self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
  //выносим за цикл то, что не изменяется rotation/(self.fr2- 2 * framestep)*2
  stepRotation = correctedRotation / (fr2 - 2.0) * 2;

  dx0Typical = correctedStepLenght / (2.0 * fr1 + fr2);
  dy0Typical = correctedSideLenght / (2.0 * fr1 + fr2);
  dobavka_x_ot_torsa = svIkA5 * 2.0 * sfMathSin(ugol_torsa) / fr2;
  //Некоторые константные настройки
  xr = xl = bodyTiltAtWalk;
  yl =  solyLandingSkew;
  yr = -solyLandingSkew;
  fase_offset = 0.7; // 1.25; //0.7;
  }


//Исполнить фазу 1
void walkPhasa1() {
  ztl = ztr = -gaitHeight;
  xtl_plan = correctedStepLenght * (0.5 - fr1 / (2.0 * fr1 + fr2)) - dobavka_x_ot_torsa * fr1 * fr2 / (2.0 * fr1 + fr2);
  for( j = 0; j < fr1; j ++ ) {
    //s = amplitude / 2 * sfMathCos( alpha01 * j / 2 );
    s = (amplitude / 2 + correctedSideLenghtHalf) * sfMathCos(alpha01 * (j / 2 + fase_offset * framestep));
    ytr = s - d10 + correctedSideLenghtHalf;// - (1 - sfMathCos(tors_angle)) * svIkA5;
    ytl = s + d10 + correctedSideLenghtHalf;// + (1 - sfMathCos(tors_angle)) * svIkA5;
    if( stepType == STEP_FIRST ) continue;
    dx1 = (xtl_plan - xtl) / (fr1 - j);
    xtr += dx1;
    xtl += dx1;
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    //testDrop();
    }
  }

//Исполнить фазу 3
void walkPhasa3() {
  ztl = ztr = -gaitHeight;
  xtr_plan = correctedStepLenght * (0.5 - fr1 / (2.0 * fr1 + fr2)) - dobavka_x_ot_torsa * fr1 * fr2 / (2.0 * fr1 + fr2);
  for( j = 0; j < fr1; j ++ ) {
    //s = amplitude / 2 * sfMathCos( alpha01 * (j + fr1) / 2 );
    s = (amplitude / 2 + correctedSideLenghtHalf) * sfMathCos(alpha01 * ((j -fr2) / 2 + fase_offset * framestep));
    ytr = s - d10 - correctedSideLenghtHalf; //- (1 - sfMathCos(tors_angle)) * svIkA5;
    ytl = s + d10 + correctedSideLenghtHalf; //+ (1 - sfMathCos(tors_angle)) * svIkA5;
    dx1 = (xtr_plan - xtr) / (fr1 - j);
    xtr += dx1;
    xtl += dx1;
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    //testDrop();
    }
  }
  
//Исполнить фазу 2
void walkPhasa2() {
  wr = 0; wl = 0;
  dy = correctedSideLenght / fr2;
  xtl_plan = correctedStepLenght * (0.5 - (fr1 + fr2) / (2.0 * fr1 + fr2)) + dobavka_x_ot_torsa * fr1 * fr2 / (2.0 * fr1 + fr2);
  xtr_plan = correctedStepLenght * 0.5 + dx0Typical + dobavka_x_ot_torsa;
  for( j = 0; j < fr2; j ++ ) {
    ztr = -gaitHeight + stepHeight;
    dx2 = (xtl_plan - xtl) / (fr2 - j);
    if (stepType == STEP_FIRST)
        tors_angle = sfFMathASin(dobavka_x_ot_torsa / svIkA5 * (j + 1) / 2);
    else
        tors_angle = sfFMathASin(sfMathSin(-ugol_torsa) + dobavka_x_ot_torsa / svIkA5 * (j + 1));
    if( j == 0 ) {
      xtr += dx2 - dobavka_x_ot_torsa;
      ytr  = s - d10 + dy0Typical;// - (1 - sfMathCos(tors_angle)) * svIkA5;
      }
    else if( j == (fr2 - 1) ) {
        xtr += dx2 - dobavka_x_ot_torsa;
      ytr  = s - d10 + 2 * dy0Typical - correctedSideLenght;// - (1 - sfMathCos(tors_angle)) * svIkA5;
      }
    else {
        if (j == 1)
            ztr = -gaitHeight + stepHeight / 2.0;
        dx = (xtr_plan - xtr) / (fr2 - j);
      xtr += dx;
      ytr  = s - 64 + dy0Typical - dy * fr2 / (fr2 - 2) * (j / 2);// - (1 - sfMathCos(tors_angle)) * svIkA5;
      // wr = correctedRotation - j * stepRotation;
      // wl = wr;
      }
    //if (correctedRotation < 0) wl = j * correctedRotation / (fr2 - 1);
    xtl += dx2;
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
  wr = 0; wl = 0;
  dy = correctedSideLenght / (fr2 - 2.0);
  if( stepType == STEP_LAST ) {
    xtr_plan = 0;
    xtl_plan = 0;
    }
  else {
     xtr_plan = correctedStepLenght * (0.5 - (fr1 + fr2) / (2.0 * fr1 + fr2)) + dobavka_x_ot_torsa * fr1 * fr2 / (2.0 * fr1 + fr2);
     xtl_plan = correctedStepLenght * 0.5 + dx0Typical + dobavka_x_ot_torsa;
    }
  for( j = 0; j < fr2; j ++ ) {
    ztl = -gaitHeight + stepHeight;
    
    if( stepType == STEP_LAST && j == (fr2 - 1) ) {
      ztl = -gaitHeight;
      ytl = s + d10;
      tors_angle = sfFMathASin(sfMathSin(ugol_torsa) - dobavka_x_ot_torsa / svIkA5 * (j + 1) / 2);
      }
    else tors_angle = sfFMathASin(sfMathSin(ugol_torsa) - dobavka_x_ot_torsa / svIkA5 * (j + 1));
    dx4 = (xtr_plan - xtr) / (fr2 - j);
    if( j == 0 ) {
      xtl += dx4 - dobavka_x_ot_torsa;
      ytl = s + d10 + dy0Typical;// + (1 - sfMathCos(tors_angle)) * svIkA5;
      ztl = -gaitHeight + stepHeight / 2.0;
      }
    else if( j == (fr2 - 1) ) {
      xtl += dx4 - dobavka_x_ot_torsa;
      ytl  = s + d10 + dy0Typical * 2.0 - correctedSideLenght;// + (1 - sfMathCos(tors_angle)) * svIkA5;
      }
    else {
      if (j == 1)
          ztl = -gaitHeight + stepHeight / 2.0;
      dx = (xtl_plan - xtl) / (fr2 - j);
      xtl += dx;
      ytl  = s + 64 + dy0Typical - dy * j / 2;// + (1 - sfMathCos(tors_angle)) * svIkA5;
      // wr = j * stepRotation - correctedRotation;
      // wl = wr;
      }
    //if (correctedRotation > 0) wr = -j * correctedRotation / (fr2 - 1);
    xtr += dx4;
    ytr += dy0Typical;
    
    //Вычислить обратную кинематику и отправить 
    if( !computeAlphaForWalk() )
      return;
    //testDrop();
    }
  }

//Здесь отдельно алгоритм медленной ходьбы
//#include <roki3ASlowWalk.cpp>


//Универсальный (для быстрой и медленной) ходьбы цикл
// if half == 1 then just half of cycle executed
// if half == 0 then full cycle executed
void walkCycle(int half) {
  //Сохраняем стопу
  float xrOld = xr;
  float yrOld = yr;
  float xlOld = xl;
  float ylOld = yl;

    //Выполнить цикл быстрой ходьбы
  walkInit();
  walkPhasa1();
  walkPhasa2();
  if (half == 0){
      walkPhasa3();
      walkPhasa4();
      }
  
  //Восстанавливаем стопу
  xr = xrOld;
  yr = yrOld;
  xl = xlOld;
  yl = ylOld;
  }

//Факторы определяют степень зависимости поворота стоп от показаний IMU
int leftRightFactor; //Фактор миксования лево-право
int frontBackFactor; //Фактор миксования вперед-назад

void mixing() {
  //Начальные установки
  leftRightFactor = 100;
  frontBackFactor = 180;
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

void runTest() {
  //Стали в начальную позу
  walkInitialPoseFine();
  // waiting for push button at head
  //while (startStop == 0) sfWaitFrame(1);
  //Выполняем первый шаг
  stepType = STEP_FIRST;
  stepLength = stepLengthOrder/ 3.0;
  walkCycle(0);
  
  //Второй разгонный шаг
  stepType = STEP_OTHER;
  stepLength = stepLengthOrder * 2.0 / 3.0;
  walkCycle(0);
  
  //Цикл одинаковых шагов
  stepLength = stepLengthOrder * 1.0;
  int i;
  for( i = 0; i < cycle_number; i++ ){
    stabilizeRotationByIMU();
    walkCycle(0);
    if(orderFromHead == 4) break;
    }
  //Финальный шаг
  stepType = STEP_LAST;
  walkCycle(0);
  
  //Reverse run
  walkFinalPoseFine();
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

void main() {
  restart_flag = 0;
  pitStop = 0;
  startStop = 0;
  slowWalk = 0;
  setup();
  sideLength = 0;
  rotation = 0;
  
  int frameCount = 80;
  sfPoseGroup(MASK_ALL, 0, frameCount );
  sfWaitFrame( frameCount );

  //Фиксируем направление "вперед"
  sfQuaternionToEulerImu();
  forwardDirection = svEulerYaw;
  
  //Запускаем миксинг
  sfCreateTask( mixing, 20 );
  while (pitStop == 0) sfWaitFrame(1); // waithing to change parameters

  svButtonRight = SV_SLOT_INACTIVE;
  svButtonLeft = SV_SLOT_INACTIVE;
  sfBip(1, 1);
  while (svButtonPress != SV_BUTTON_RIGHT_PRESS) sfWaitFrame(1); // waithing to change parameters
  svButtonRight = SV_SLOT_RESTART_RUN;
  svButtonLeft = SV_SLOT_RELAX;
  restart_flag = 1;
  
  runTest(); // 
  
  //Переходим к срипту "Напряжен"
  //sfStartSlot( sfSlotIndex("androSotTenseUp.cpp") );
  }

