/*
  Проект "Roki-2"
  Автор
    Зубр ОВК
  Описание
    Скрипт движок ходьбы по методу Азера 
*/
#include <roki2met.h>
#include <roki2global.h>

int IKerr;
float stepLength; // = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
float sideLength; // = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
float rotation; // = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.

float selfMotionShiftCorrectionX; // = -self.glob.params['MOTION_SHIFT_TEST_X'] / 21
float selfMotionShiftCorrectionY; // = -self.glob.params['MOTION_SHIFT_TEST_Y'] / 21
float selfFirstStepYield; // = self.glob.first_step_yield
float selfCycleStepYield; // = self.glob.cycle_step_yield
float selfSideStepRightYield; // = self.glob.side_step_right_yield
float selfSideStepLeftYield; // = self.glob.side_step_left_yield
int   selfFirstLegIsRightLeg; // = True
int   motion_to_right;
float   side_motion;
int   selfInitPoses; // = 400//self.simThreadCycleInMs

float rotationYieldRight;
float rotationYieldLeft;

int   framestep;
int   x_cap;
int   y_cap;
float xtr; // = 0
float xtr_cap;
float xr_cap;
float ytr; // = -self.d10   #-53.4
float ytr_cap;
float ztr; // = -self.gaitHeight

float xr; // = 0
float yr; // = 0
float zr; // = -1
float wr; // = 0

float xtl; // = 0
float xtl_cap;
float xl_cap;
float xt_cap_limit;
float x_cap_limit;
float ytl; // = self.d10   # 53.4
float ytl_cap;
float side_cap;
float yt_cap_limit;
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
int walking_frame;

float bodyTiltAtWalk;
float solyLandingSkew;

int stepNumber;
float yaw;
float pitch;
float roll;

void quaternion_to_euler_angle() {
    float w = svImuQuaterW / 16384.0;
    float x = svImuQuaterX / 16384.0;
    float y = svImuQuaterY / 16384.0;
    float z = svImuQuaterZ / 16384.0;
    float t0 = +2.0 * (w * x + y * z);
    float t1 = +1.0 - 2.0 * (x * x + y * y);
    float t2 = +2.0 * (w * y - z * x);
    t2 = (t2 > +1.0 ? 1.0 : t2);
    t2 = (t2 < -1.0 ? -1.0 : t2);
    float t3 = +2.0 * (w * z + x * y);
    float t4 = +1.0 - 2.0 * (y * y + z * z);
    pitch = sfFMathATan2(t0, t1) - 1.5708;
    roll = sfFMathASin(t2);
    yaw = sfFMathATan2(t3, t4);
}

float get_yaw() {
    quaternion_to_euler_angle();
    return yaw;
}

void setup() {
  rotationYieldRight = 0.23;
  rotationYieldLeft = 0.23;
  //Установка параметров модели для инверсной кинематики
  svIkA5 = 40.2; // мм расстояние от оси симметрии до оси сервы 5
  svIkB5 = 0; // мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
  svIkC5 = 0;    // мм расстояние от оси сервы 6 до нуля Z по вертикали
  svIkA6 = 0;   // мм расстояние от оси сервы 6 до оси сервы 7
  svIkA7 = 99;   // мм расстояние от оси сервы 7 до оси сервы 8
  svIkA8 = 99;   // мм расстояние от оси сервы 8 до оси сервы 9
  svIkA9 = 0;   // мм расстояние от оси сервы 9 до оси сервы 10
  svIkA10 = 13.7;  // мм расстояние от оси сервы 10 до центра стопы по горизонтали
  svIkB10 = 23.8;  // мм расстояние от оси сервы 10 до низа стопы
  svIkC10 = 0;   // мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
  
  e10 = 55;      // мм половина длины стопы
  d10 = 62;      // расстояние по Y от центра стопы до оси робота
  
  //Ограничения движения моторов в градусах относительно нулевого положения
  svIkLimA5min = -150 * MATH_RAD_PER_DEGREE;
  svIkLimA5max = 150 * MATH_RAD_PER_DEGREE;
  
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
  
  gaitHeight = 170;  // Distance between Center of mass and floor in walk pose
  stepHeight = 40.0; // elevation of sole over floor
  
  selfMotionShiftCorrectionX = 50.0 / 21.0;
  selfMotionShiftCorrectionY = 0.0 / 21.0;


  ztr0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -220.8
  ztl0 = -(svIkC5+svIkA6+svIkA7+svIkA8+svIkA9+svIkB10 - 1); // -220.8
  
  x_cap = 1; // enable capture steps along x
  y_cap = 1; // enable capture steps along y
  side_cap = 0;
  xtr_cap = 0; // right leg capture addon
  xtl_cap = 0; // left leg capture addon
  ytr_cap = 0; // right leg capture addon
  ytl_cap = 0; // left leg capture addon
  xt_cap_limit = 30.0;
  yt_cap_limit = 30.0;
  xr_cap = 0;         // right foot pitch correction
  xl_cap = 0;         // left foot pitch correction
  x_cap_limit = 0.17;
    
  zr = zl = -1;
    
  stepLength = 0.0;    // -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
  sideLength = 0.0;    // -20 - +20. Side step length to right (+) and to left (-)
  rotation = 0;        // -45 - +45 degrees Centigrade per step + CW, - CCW.
  selfFirstLegIsRightLeg = 1; // = True
  motion_to_right = 1;
  side_motion = 0.0;
  selfInitPoses = 10; //self.simThreadCycleInMs

  stepZtr = (ztr0 + gaitHeight) / selfInitPoses;
  stepZtl = (ztl0 + gaitHeight) / selfInitPoses;
  
  bodyTiltAtWalk = 0.01;
  solyLandingSkew = 0.01;

  fr1 = 8; //8;           // frame number for 1-st phase of gait ( two legs on floor)
  fr2 = 12; //12;          // frame number for 2-nd phase of gait ( one leg in air)
  amplitude = 20;    // mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2

  stepYtr = amplitude / 2.0 / selfInitPoses;
  stepYtl = amplitude / 2.0 / selfInitPoses;
  alpha01 = MATH_PI;
  if( fr1 != 0 )
    alpha01 = alpha01 * 2 / fr1;
  
  //Длительность шага алгоритма
  //На каждом шаге происходит линейная интерполяция позиций в течение количества
  //фреймов, заданных данной переменной
  //Фактическая длительность шага составляет fps - 1, т.е. она короче на один фрейм
  fps = 5;
  walking_frame = 2;
  }
  

int computeAlphaForWalk() {
    //Вычислить значения углов серв для правой ноги
    if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
    else sfIkAngle(xtl, -ytl, ztl, xl, -yl, zl, -wl);
    if (svIkOutPresent) {
        //Записать новые значения в сервы
            sfPoseGroupLin(MASK_RIGHT_PELVIC, -svIkEncA5, fps);
            sfPoseGroupLin(MASK_RIGHT_HIP_SIDE, -svIkEncA6, fps);
            sfPoseGroupLin(MASK_RIGHT_HIP, -svIkEncA7, fps);
            sfPoseGroupLin(MASK_RIGHT_KNEE, -svIkEncA8 / 2, fps);
            sfPoseGroupLin(MASK_RIGHT_KNEE_BOT, -svIkEncA8 / 2, fps);
            sfPoseGroupLin(MASK_RIGHT_FOOT_FRONT, svIkEncA9, fps);
            sfPoseGroupLin(MASK_RIGHT_FOOT_SIDE, -svIkEncA10, fps);
    	  sfPoseGroupLin( MASK_RIGHT_ELBOW, 5145, fps);
    }
    else IKerr++;
    //Вычислить значение углов серв для левой ноги
    if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
    else sfIkAngle(xtr, ytr, ztr, xr, yr, zr, -wr);
    if (svIkOutPresent) {
        //Записать новые значения в сервы
            sfPoseGroupLin(MASK_LEFT_PELVIC, -svIkEncA5, fps);
            sfPoseGroupLin(MASK_LEFT_HIP_SIDE, -svIkEncA6, fps);
            sfPoseGroupLin(MASK_LEFT_HIP, -svIkEncA7, fps);
            sfPoseGroupLin(MASK_LEFT_KNEE, -svIkEncA8 / 2, fps);
            sfPoseGroupLin(MASK_LEFT_KNEE_BOT, -svIkEncA8 / 2, fps);
            sfPoseGroupLin(MASK_LEFT_FOOT_FRONT, svIkEncA9, fps);
            sfPoseGroupLin(MASK_LEFT_FOOT_SIDE, -svIkEncA10, fps);
            sfPoseGroupLin(MASK_LEFT_ELBOW, 5145, fps);
    }
    else IKerr++;
    //Ожидать когда движение завершится
    sfWaitFrame(walking_frame);
    return 1;
}

void computeAlphaForWalkFine(int frames) {
    //Вычислить значения углов серв для правой ноги
    if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtr, ytr, ztr, xr, yr, zr, wr );
    else sfIkAngle(xtl, -ytl, ztl, xl, -yl, zl, -wl);
    //Ни одного решения не найдено
    if (svIkOutPresent) {
        //Записать новые значения в сервы
            sfPoseGroupLin(MASK_RIGHT_PELVIC, -svIkEncA5, frames);
            sfPoseGroupLin(MASK_RIGHT_HIP_SIDE, -svIkEncA6, frames);
            sfPoseGroupLin(MASK_RIGHT_HIP, -svIkEncA7, frames);
            sfPoseGroupLin(MASK_RIGHT_KNEE, -svIkEncA8 / 2, frames);
            sfPoseGroupLin(MASK_RIGHT_KNEE_BOT, -svIkEncA8 / 2, frames);
            sfPoseGroupLin(MASK_RIGHT_FOOT_FRONT, svIkEncA9, frames);
            sfPoseGroupLin(MASK_RIGHT_FOOT_SIDE, -svIkEncA10, frames);
            sfPoseGroupLin( MASK_RIGHT_ELBOW, 5145, frames);
    }
    else IKerr++;
    //Вычислить значение углов серв для левой ноги
    if (selfFirstLegIsRightLeg == 1) sfIkAngle( xtl, -ytl, ztl, xl, -yl, zl, wl );
    else sfIkAngle(xtr, ytr, ztr, xr, yr, zr, -wr);
    if (svIkOutPresent) {
        //Записать новые значения в сервы
            sfPoseGroupLin(MASK_LEFT_PELVIC, -svIkEncA5, frames);
            sfPoseGroupLin(MASK_LEFT_HIP_SIDE, -svIkEncA6, frames);
            sfPoseGroupLin(MASK_LEFT_HIP, -svIkEncA7, frames);
            sfPoseGroupLin(MASK_LEFT_KNEE, -svIkEncA8 / 2, frames);
            sfPoseGroupLin(MASK_LEFT_KNEE_BOT, -svIkEncA8 / 2, frames);
            sfPoseGroupLin(MASK_LEFT_FOOT_FRONT, svIkEncA9, frames);
            sfPoseGroupLin(MASK_LEFT_FOOT_SIDE, -svIkEncA10, frames);
            sfPoseGroupLin(MASK_LEFT_ELBOW, 5145, frames);
    }
    else IKerr++;
    //Ожидать когда движение завершится
    sfWaitFrame(frames);
}

int j;

//Установить начальную позу
//Если робот был не исходной позе (ровный на высоте ztr0), то робота дергает
void walkInitialPose() {
  xtr = xtl = 0;
  xtr_cap = xtl_cap = 0;
  for( j = 0; j < selfInitPoses; j++ ) {
    //Задать положение стоп для двух ног
    ztr = ztr0 - j * stepZtr;
    ztl = ztl0 - j * stepZtl;
    ytr = -d10 - j * stepYtr;
    ytl =  d10 - j * stepYtl;
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalk();
    }
  }


//Установить начальную позу
//Алгоритм с использованием синусоидального движения
//Мы вычисляем конечную позу и за заданное количество фреймов робот плавно ее достигает
//вне зависимости от исходного состояния
void walkInitialPoseFine() {
  xtr = xtl = 0;
  ztr = ztl = -gaitHeight;
  ytr = -d10 - amplitude / 4.0;
  ytl =  d10 - amplitude / 4.0;
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
    if (j == framestep && x_cap ) xl = bodyTiltAtWalk;
    s = amplitude * (0.5 - (j * 1.0 + framestep) / fr1);
    ytr = s - d10 + correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    if( stepType == STEP_FIRST ) continue;
    xtl = xtl0 - dx0Typical - dx0Typical * j / framestep + xtl_cap;
    xtr = xtr0 - dx0Typical - dx0Typical * j / framestep + xtr_cap;
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalk();
    //if (j == 0) {
        //if (selfFirstLegIsRightLeg == 1) sfFreeGroup(MASK_LEFT_FOOT_SIDE | MASK_LEFT_FOOT_FRONT);
        //else sfFreeGroup(MASK_RIGHT_FOOT_SIDE | MASK_RIGHT_FOOT_FRONT);
    //}
    }
  }

//Исполнить фазу 3
void walkPhasa3() {
  ztl = ztr = -gaitHeight;
  for( j = 0; j < fr1; j += framestep ) {
    if (j == framestep && x_cap) xr = bodyTiltAtWalk;
    s = -amplitude * (0.5 - (j * 1.0 + framestep) / fr1);
    ytr = s - d10 - correctedSideLenghtHalf;
    ytl = s + d10 + correctedSideLenghtHalf;
    xtl -= dx0Typical;
    xtr -= dx0Typical;
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalk();
    //if (j == 0) {
        //if (selfFirstLegIsRightLeg == 1) sfFreeGroup(MASK_RIGHT_FOOT_SIDE | MASK_RIGHT_FOOT_FRONT );
        //else sfFreeGroup(MASK_LEFT_FOOT_SIDE | MASK_LEFT_FOOT_FRONT);
    //}
    }
  }
  
float dx,dy;
float stepRotation;

//Исполнить фазу 2
void walkPhasa2() {
  ztr = -gaitHeight + stepHeight;
  dy = correctedSideLenght * framestep / (fr2 - 2.0 * framestep);
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
      ytr  = s - d10 + 2 * dy0Typical - correctedSideLenght + ytr_cap;
      }
    else {
      xtr += dx;
      if (j == (fr2 - framestep * 2) && x_cap) {
          quaternion_to_euler_angle();
          xtr_cap = (pitch > 0.0 ? pitch * gaitHeight : 0.0);
          xtr_cap = (xtr_cap > xt_cap_limit ? xt_cap_limit : xtr_cap);
          xtr += xtr_cap;
          xr_cap = (pitch > 0.0 ? pitch : 0.0);
          xr_cap = (xr_cap > x_cap_limit ? x_cap_limit : xr_cap);
          xr += xr_cap;
      }
      //if (j == (fr2 - framestep * 2) && y_cap) {
      //    quaternion_to_euler_angle();
      //    ytr_cap = (roll < 0.0 ? roll * gaitHeight : 0.0);
      //    ytr_cap = (ytr_cap < -yt_cap_limit ? -yt_cap_limit : ytr_cap);
      //    ytr += ytr_cap;
      //}
      ytr  = s - 64 + dy0Typical - dy * (j / 2);
      wr = correctedRotation - j * stepRotation;
      wl = wr;
      }
    xtl -= dx0Typical;
    ytl += dy0Typical;

    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalk();
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
    else if( j == (fr2 - framestep)) {
      xtl -= dx0;
      ytl  = s + d10 + correctedSideLenghtHalf;
      }
    else {
      xtl += dx;
      if (j == (fr2 - framestep * 2) && x_cap) {
          quaternion_to_euler_angle();
          xtl_cap = (pitch > 0.0 ? pitch * gaitHeight : 0.0);
          xtl_cap = (xtl_cap > xt_cap_limit ? xt_cap_limit : xtl_cap);
          xtl += xtl_cap;
          xl_cap = (pitch > 0.0 ? pitch : 0.0);
          xl_cap = (xl_cap > x_cap_limit ? x_cap_limit : xl_cap);
          xl += xl_cap;
      }
      ytl  = s + 64 + correctedSideLenghtHalf;
      wr = j * stepRotation - correctedRotation;
      wl = wr;
      }
    xtr -= dx0;
    ytr += dy0Typical;
    
    //Вычислить обратную кинематику и отправить 
    computeAlphaForWalk();
    }
  }

float forwardDirection;

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

void stabilizeRotationByIMU(){
  //Корректировать направление
  sfQuaternionToEulerImu();
  //rotation = -(svEulerYaw - forwardDirection);
  rotation = forwardDirection - get_yaw();
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

void kick(int invert) {
    float gaitHeightKick = 200;
    float stepHeightKick = 40;
    float stepLengthKick = 50 * invert;
    float kick_size = kick_power * invert;
    float bodyTiltAtKick = 0.04;
    int pose_taking_cycles = 20;
    //Сохраняем стопу
    float xrOld = xr;
    float yrOld = yr;
    float xlOld = xl;
    float ylOld = yl;
    //if (small == 1) kick_size = -10;
    selfFirstLegIsRightLeg = kick_by_right;
    dx0Typical = stepLengthKick / (2 * fr1 + fr2 + 2.0 * framestep) * framestep;
    xr = xl = bodyTiltAtKick;
    yl = solyLandingSkew;
    yr = -solyLandingSkew;
    s = (amplitude / 2) * sfMathCos(alpha01 * (fr1 / 2 + 0.5 * framestep));
    xtr = xtl = 0;

    // initial pose
    ztr = ztl = -gaitHeightKick;
    //ytr = -d10 - amplitude*3 ;
    //ytl =  d10 - amplitude*3 ;
    //computeAlphaForWalkFine( 40 );
    for (j = 0; j < pose_taking_cycles; j += 1) {
        ztr = ztr0 + j * (-ztr0 - gaitHeightKick) / pose_taking_cycles;
        ztl = ztr;
        ytr = -d10 - j * amplitude * 3 / pose_taking_cycles;
        ytl = ytr + 2 * d10;
        computeAlphaForWalk();
    }

    ztr = ztl = -gaitHeightKick;
    ytr = s - d10;
    ytl = s + d10;

    for (j = fr1; j < fr1 + fr2; j += framestep) {
        ztr = -gaitHeightKick + stepHeightKick;
        dx = stepLengthKick / fr2;
        dx0 = stepLengthKick / (2 * fr1 + fr2 + 4) * framestep;
        if (j == fr1 || j == fr1 + fr2 - 2) {
            xtr -= dx0;
            ytr = s - (64 + kick_offset);
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
    for (j = fr1 + fr2; j < 2 * fr1 + fr2; j += framestep) {
        s = (amplitude / 2) * sfMathCos(alpha01 * (j - fr2 + framestep) / 2);
        ytr = s - d10;
        ytl = s + d10;
        ztl = -gaitHeightKick;
        ztr = -gaitHeightKick;
        dx0 = dx0Typical;
        xtl -= dx0;
        xtr -= dx0;
        computeAlphaForWalk();
    }
    for (j = 2 * fr1 + fr2; j < 2 * (fr1 + fr2); j += framestep) {
        ztl = -gaitHeightKick + stepHeightKick;
        dx0 = dx0Typical * 4 / fr2;
        dx = (stepLengthKick * (fr1 + fr2) / (4 * fr1) + 2 * dx0) / (fr2 - 2 * framestep) * framestep;
        if (j == (2 * fr1 + 2 * fr2 - framestep)) {
            ztl = -gaitHeightKick;
            ytl = s + d10;
        }
        if (j == (2 * fr1 + fr2) || j == (2 * (fr1 + fr2) - framestep)) {
            xtl -= dx0;
            ytl = s + 64;
        }
        else
            xtl += dx;
        xtr -= dx0;
        computeAlphaForWalk();
    }

    // walk_Final_Pose_After_Kick
    pose_taking_cycles = 5;
    int pose_hight_after_kick = -130; //ztr0;
    for (j = 0; j < pose_taking_cycles; j += 1) {
        ztr = -gaitHeightKick + (j + 1) * (pose_hight_after_kick + gaitHeightKick) / pose_taking_cycles;
        ztl = ztr;
        ytr = -d10 - (pose_taking_cycles - (j + 1)) * amplitude / 2 / pose_taking_cycles;
        ytl = d10 - (pose_taking_cycles - (j + 1)) * amplitude / 2 / pose_taking_cycles;
        computeAlphaForWalk();
    }
    pose_taking_cycles = 50;
    for (j = 0; j < pose_taking_cycles; j += 1) {
        ztr = pose_hight_after_kick + (j + 1) * (ztr0 - pose_hight_after_kick) / pose_taking_cycles;
        ztl = ztr;
        computeAlphaForWalk();
    }
    //Восстанавливаем стопу
    xr = xrOld;
    yr = yrOld;
    xl = xlOld;
    yl = ylOld;
}

#define CYCLE_COUNT 15 //Количество шагов. Общее количество шагов будет +3
#define STEP_LENGHT 70

void runTest() {
  //Стали в начальную позу
  //walkInitialPoseFine();
  walkInitialPose();
  //Выполняем первый шаг
  stepType = STEP_FIRST;
  stepLength = STEP_LENGHT / 3.0;
  walkCycle(0);
  
  //Второй разгонный шаг
  stepType = STEP_OTHER;
  stepLength = STEP_LENGHT * 2.0 / 3.0;
  walkCycle(0);
  
  //Цикл одинаковых шагов
  stepLength = STEP_LENGHT;
  int i;
  for( i = 0; i < CYCLE_COUNT; i++ ){
    stabilizeRotationByIMU();
    if (sfAbs(roll) > 0.1 && y_cap) {
        side_cap = roll * gaitHeight * 2;
    }
    else side_cap = 0;
    sideLength = side_cap;
    walkCycle(0);
    }

  stepLength = STEP_LENGHT * 2.0 / 3.0;
  walkCycle(0);
  //Финальный шаг
  stepLength = STEP_LENGHT / 3.0;
  stepType = STEP_LAST;
  walkCycle(0);
  
  //Становимся в исходную позу
  walkFinalPoseFine();
  }

void main() {
  setup();
  sideLength = 0;
  rotation = 0;
  IKerr = 0;

   //Фиксируем направление "вперед"
  sfQuaternionToEulerImu();
  //forwardDirection = svEulerYaw;
  forwardDirection = get_yaw();
  
  //Запускаем миксинг
  sfCreateTask( mixing, 20 );

  kick(1);
  //runTest(); // простой тест ходьбы прямо
  
  //Переходим к срипту "Напряжен"
  sfStartSlot( sfSlotIndex("roki2TenseUp.cpp") );
  }

