
//IronArt v0.117
//Данный файл сгенерирован автоматически системой IronArt и не должен редактироваться
//
#ifndef ROKI2MET_H
#define ROKI2MET_H
#include <Sv8Sys.h>

//Средние значения акселерометра
int svImuAccX         _at_ 2560;
int svImuAccY         _at_ 2561;
int svImuAccZ         _at_ 2562;
//Сырые значения акселерометра непосредственно с сенсора
int svImuAccRawX      _at_ 2563;
int svImuAccRawY      _at_ 2564;
int svImuAccRawZ      _at_ 2565;

//Средние значения гироскопа
int svImuGyroX        _at_ 2566;
int svImuGyroY        _at_ 2567;
int svImuGyroZ        _at_ 2568;
//Сырые значения гироскопа непосредственно с сенсора
int svImuGyroRawX     _at_ 2569;
int svImuGyroRawY     _at_ 2570;
int svImuGyroRawZ     _at_ 2571;

//Значения кватерниона
int svImuQuaterX      _at_ 2572;
int svImuQuaterY      _at_ 2573;
int svImuQuaterZ      _at_ 2574;
int svImuQuaterW      _at_ 2575;
int svImuQuaterAcc    _at_ 2576;

//Медленные значения акселерометра
int svImuAccSlowX     _at_ 2577;
int svImuAccSlowY     _at_ 2578;
int svImuAccSlowZ     _at_ 2579;


//Напряжение питания
int svAdc             _at_ 2580;

//Реакция на кнопки
//Запишите сюда номер слота, который должен выполняться при нажатии на кнопку
int svButtonLeft      _at_ 2581; //На левой кнопке по умолчанию расслабление
int svButtonRight     _at_ 2582; //На правой кнопке по умолчанию напряжение
int svButtonPress     _at_ 2583; //Состояние нажатых кнопок

//Маскирование управления движками от головы
int svHeadControlMask _at_ 2584;

//Количество фреймов при прямой записи целевого угла в движок
int svDefFrameCount   _at_ 2585;

//Углы Эйлера, получаемые после конвертации с помощью функции sfQuaternionToEuler
float svEulerYaw      _at_ 2600;
float svEulerPitch    _at_ 2601;
float svEulerRoll     _at_ 2602;


#define SV_BUTTON_LEFT_PRESS  1 //Битовая маска для левой кнопки
#define SV_BUTTON_RIGHT_PRESS 2 //Битовая маска для правой кнопки




//Управление от внешнего пульта
int svRemoteStatus         _at_ 3250;  //Состояние пульта (подключен или нет)
int svRemoteButton         _at_ 3251;  //Состояние кнопок пульта (битовое поле, 0 - не нажата, 1 - нажата)
int svRemoteLeftJoystick   _at_ 3252;  //Кнопка левого джойстика
int svRemoteLeftJoystickX  _at_ 3253;  //Смещение левого джойстика по X
int svRemoteLeftJoystickY  _at_ 3254;  //Смещение левого джойстика по Y
int svRemoteRightJoystick  _at_ 3255;  //Кнопка правого джойстика
int svRemoteRightJoystickX _at_ 3256;  //Смещение правого джойстика по X
int svRemoteRightJoystickY _at_ 3257;  //Смещение правого джойстика по Y

//Константы кнопок Android-пульта (AndroidRemoteButtons)
//Аппаратные кнопки
#define ARB_A                    1
#define ARB_B                    2
#define ARB_X                    4
#define ARB_Y                    8
#define ARB_L1                  16
#define ARB_R1                  32
#define ARB_L2                  64
#define ARB_R2                 128
#define ARB_LEFT_JOYSTICK      256
#define ARB_RIGHT_JOYSTICK     512
#define ARB_SELECT            1024
#define ARB_START             2048
//Нажатия pad-а генерит также левый джойстик, когда его значения превышают некий порог
#define ARB_LEFT_PAD_UP       4096
#define ARB_LEFT_PAD_RIGHT    8192
#define ARB_LEFT_PAD_DOWN    16384
#define ARB_LEFT_PAD_LEFT    32768

//Программные кнопки
#define ARB_WALK_FAST        65536
#define ARB_WALK_SLOW       131072
#define ARB_KICK_RIGHT      262144
#define ARB_KICK_LEFT       524288
#define ARB_GETUP_STOMACH  1048576
#define ARB_GETUP_FACEUP   2097152
#define ARB_APPLAUSE       4194304
#define ARB_BEEP           8388608



//Отладочные переменные
int svDebug0         _at_ 2603;
int svDebug1         _at_ 2604;
int svDebug2         _at_ 2605;
int svDebug3         _at_ 2606;




//Индексы слотов, которые можно применять в svButtonLeft, svButtonRight
#define SV_SLOT_MAX            64 //Максимальное количество слотов для скриптов
#define SV_SLOT_STOP           65 //Остановить текущий скрипт
#define SV_SLOT_RESTART_RUN    66 //Перезапустить текущий скрипт и сразу стартовать
#define SV_SLOT_RESTART_PAUSE  67 //Перезапустить текущий скрипт в режиме паузы
#define SV_SLOT_ZERO_POS       68 //Перейти в нулевую позицию
#define SV_SLOT_RELAX          69 //Расслабить моторы
#define SV_SLOT_INACTIVE       -1 //Нет операции со слотом




//Модель гуманоидного робота для вычисления прямой и обратной кинематики
int svHumoModelDistPelvic       _at_ 3264;  //Расстояние от центра до точки поворота таза
int svHumoModelLengthPelvicHip  _at_ 3265;  //Расстояние от точки поворота таза до поворота бедра вбок
int svHumoModelLengthHipHip     _at_ 3266;  //Расстояние между осями поворота бедра вбок и вперед
int svHumoModelLengthHipKnee    _at_ 3267;  //Длина бедра до колена
int svHumoModelLengthKneeFoot   _at_ 3268;  //Длина голени от колена до поворота стопы вперед
int svHumoModelLengthFootFoot   _at_ 3269;  //Расстояние между осями поворота стопы вбок и вперед
int svHumoModelDistFootFloor    _at_ 3270;  //Расстояние между осью поворота стопы вбок и землей
int svHumoModelFootLength       _at_ 3271;  //Длина стопы
int svHumoModelFootWidth        _at_ 3272;  //Ширина стопы
int svHumoModelFootCenterBack   _at_ 3273; //Расстояние от задника стопы до оси поворота
int svHumoModelFootCenterInner  _at_ 3274; //Расстояние от внутренней части стопы до оси поворота

int svHumoModelAngPelvic        _at_ 3258; //Угол поворота таза
int svHumoModelAngHipSide       _at_ 3260; //Угол поворота бедра вбок
int svHumoModelAngHip           _at_ 3259; //Угол поворота бедра вперед
int svHumoModelAngKnee          _at_ 3261; //Угол поворота колена
int svHumoModelAngFoot          _at_ 3262; //Угол поворота ступни вперед
int svHumoModelAngFootSide      _at_ 3263; //Угол поворота ступни вбок



//Система вычисления обратной кинематики по методу Азера
float svIkA5   _at_ 3275;  // мм расстояние от оси симметрии до оси сервы 5
float svIkB5   _at_ 3276;  // мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
float svIkC5   _at_ 3277;  // мм расстояние от оси сервы 6 до нуля Z по вертикали
float svIkA6   _at_ 3278;  // мм расстояние от оси сервы 6 до оси сервы 7
float svIkA7   _at_ 3279;  // мм расстояние от оси сервы 7 до оси сервы 8
float svIkA8   _at_ 3280;  // мм расстояние от оси сервы 8 до оси сервы 9
float svIkA9   _at_ 3281;  // мм расстояние от оси сервы 9 до оси сервы 10
float svIkA10  _at_ 3282;  // мм расстояние от оси сервы 10 до центра стопы по горизонтали
float svIkB10  _at_ 3283;  // мм расстояние от оси сервы 10 до низа стопы
float svIkC10  _at_ 3284; // мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали

//Лимиты вращения двигателей в радианах
float svIkLimA5min  _at_ 3285;
float svIkLimA5max  _at_ 3286;
float svIkLimA6min  _at_ 3287;
float svIkLimA6max  _at_ 3288;
float svIkLimA7min  _at_ 3289;
float svIkLimA7max  _at_ 3290;
float svIkLimA8min  _at_ 3291;
float svIkLimA8max  _at_ 3292;
float svIkLimA9min  _at_ 3293;
float svIkLimA9max  _at_ 3294;
float svIkLimA10min _at_ 3295;
float svIkLimA10max _at_ 3296;

int svIkOutPresent  _at_ 3297; //Флаг наличия результатов вычисления, 0 - когда нету решения

int svIkEncA5       _at_ 3298; //Угол поворота таза
int svIkEncA6       _at_ 3299; //Угол поворота бедра вбок
int svIkEncA7       _at_ 3300; //Угол поворота бедра вперед
int svIkEncA8       _at_ 3301; //Угол поворота колена
int svIkEncA9       _at_ 3302; //Угол поворота ступни вперед
int svIkEncA10      _at_ 3303; //Угол поворота ступни вбок


//Ссылки для прямого доступа к двигателям
// motorTarget   - целевой угол двигателя или ШИМ в зависимости от режима двигателя
// motorCurrent  - текущий фактический угол поворота выходного вала двигателя
// motorPwm      - текущий ШИМ, подаваемый на двигатель
// motorVelo     - текущая скорость двигателя
// motorFlags    - текущие флаги двигателя
// motorAddonMix - добавочная величина, которая добавляется при к целевому углу двигателя
//где motor - имя двигателя в таблице юнитов
//Поворот головы
int headRotateTarget _at_ 2610;
int headRotateCurrent _at_ 2611;
int headRotatePwm _at_ 2612;
int headRotateVelo _at_ 2613;
int headRotateFlags _at_ 2615;
int headRotateAddonMix _at_ 2617;
//Поворот торса
int torsoRotateTarget _at_ 2626;
int torsoRotateCurrent _at_ 2627;
int torsoRotatePwm _at_ 2628;
int torsoRotateVelo _at_ 2629;
int torsoRotateFlags _at_ 2631;
int torsoRotateAddonMix _at_ 2633;
//Правая ключица
int rightClavicleTarget _at_ 2642;
int rightClavicleCurrent _at_ 2643;
int rightClaviclePwm _at_ 2644;
int rightClavicleVelo _at_ 2645;
int rightClavicleFlags _at_ 2647;
int rightClavicleAddonMix _at_ 2649;
//Левая ключица
int leftClavicleTarget _at_ 2658;
int leftClavicleCurrent _at_ 2659;
int leftClaviclePwm _at_ 2660;
int leftClavicleVelo _at_ 2661;
int leftClavicleFlags _at_ 2663;
int leftClavicleAddonMix _at_ 2665;
//Правое плечо
int rightShoulderTarget _at_ 2674;
int rightShoulderCurrent _at_ 2675;
int rightShoulderPwm _at_ 2676;
int rightShoulderVelo _at_ 2677;
int rightShoulderFlags _at_ 2679;
int rightShoulderAddonMix _at_ 2681;
//Левое плечо
int leftShoulderTarget _at_ 2690;
int leftShoulderCurrent _at_ 2691;
int leftShoulderPwm _at_ 2692;
int leftShoulderVelo _at_ 2693;
int leftShoulderFlags _at_ 2695;
int leftShoulderAddonMix _at_ 2697;
//Правый локоть вбок
int rightElbowSideTarget _at_ 2706;
int rightElbowSideCurrent _at_ 2707;
int rightElbowSidePwm _at_ 2708;
int rightElbowSideVelo _at_ 2709;
int rightElbowSideFlags _at_ 2711;
int rightElbowSideAddonMix _at_ 2713;
//Левый локоть вбок
int leftElbowSideTarget _at_ 2722;
int leftElbowSideCurrent _at_ 2723;
int leftElbowSidePwm _at_ 2724;
int leftElbowSideVelo _at_ 2725;
int leftElbowSideFlags _at_ 2727;
int leftElbowSideAddonMix _at_ 2729;
//Правый локоть вперед
int rightElbowTarget _at_ 2738;
int rightElbowCurrent _at_ 2739;
int rightElbowPwm _at_ 2740;
int rightElbowVelo _at_ 2741;
int rightElbowFlags _at_ 2743;
int rightElbowAddonMix _at_ 2745;
//Левый локоть вперед
int leftElbowTarget _at_ 2754;
int leftElbowCurrent _at_ 2755;
int leftElbowPwm _at_ 2756;
int leftElbowVelo _at_ 2757;
int leftElbowFlags _at_ 2759;
int leftElbowAddonMix _at_ 2761;
//Правый таз
int rightPelvicTarget _at_ 2770;
int rightPelvicCurrent _at_ 2771;
int rightPelvicPwm _at_ 2772;
int rightPelvicVelo _at_ 2773;
int rightPelvicFlags _at_ 2775;
int rightPelvicAddonMix _at_ 2777;
//Левый таз
int leftPelvicTarget _at_ 2786;
int leftPelvicCurrent _at_ 2787;
int leftPelvicPwm _at_ 2788;
int leftPelvicVelo _at_ 2789;
int leftPelvicFlags _at_ 2791;
int leftPelvicAddonMix _at_ 2793;
//Правое бедро вбок
int rightHipSideTarget _at_ 2802;
int rightHipSideCurrent _at_ 2803;
int rightHipSidePwm _at_ 2804;
int rightHipSideVelo _at_ 2805;
int rightHipSideFlags _at_ 2807;
int rightHipSideAddonMix _at_ 2809;
//Левое бедро вбок
int leftHipSideTarget _at_ 2818;
int leftHipSideCurrent _at_ 2819;
int leftHipSidePwm _at_ 2820;
int leftHipSideVelo _at_ 2821;
int leftHipSideFlags _at_ 2823;
int leftHipSideAddonMix _at_ 2825;
//Правое бедро
int rightHipTarget _at_ 2834;
int rightHipCurrent _at_ 2835;
int rightHipPwm _at_ 2836;
int rightHipVelo _at_ 2837;
int rightHipFlags _at_ 2839;
int rightHipAddonMix _at_ 2841;
//Левое бедро
int leftHipTarget _at_ 2850;
int leftHipCurrent _at_ 2851;
int leftHipPwm _at_ 2852;
int leftHipVelo _at_ 2853;
int leftHipFlags _at_ 2855;
int leftHipAddonMix _at_ 2857;
//Правое колено
int rightKneeTarget _at_ 2866;
int rightKneeCurrent _at_ 2867;
int rightKneePwm _at_ 2868;
int rightKneeVelo _at_ 2869;
int rightKneeFlags _at_ 2871;
int rightKneeAddonMix _at_ 2873;
//Левое колено
int leftKneeTarget _at_ 2882;
int leftKneeCurrent _at_ 2883;
int leftKneePwm _at_ 2884;
int leftKneeVelo _at_ 2885;
int leftKneeFlags _at_ 2887;
int leftKneeAddonMix _at_ 2889;
//Правая стопа вперед
int rightFootFrontTarget _at_ 2898;
int rightFootFrontCurrent _at_ 2899;
int rightFootFrontPwm _at_ 2900;
int rightFootFrontVelo _at_ 2901;
int rightFootFrontFlags _at_ 2903;
int rightFootFrontAddonMix _at_ 2905;
//Левая стопа вперед
int leftFootFrontTarget _at_ 2914;
int leftFootFrontCurrent _at_ 2915;
int leftFootFrontPwm _at_ 2916;
int leftFootFrontVelo _at_ 2917;
int leftFootFrontFlags _at_ 2919;
int leftFootFrontAddonMix _at_ 2921;
//Правая стопа вбок
int rightFootSideTarget _at_ 2930;
int rightFootSideCurrent _at_ 2931;
int rightFootSidePwm _at_ 2932;
int rightFootSideVelo _at_ 2933;
int rightFootSideFlags _at_ 2935;
int rightFootSideAddonMix _at_ 2937;
//Левая стопа вбок
int leftFootSideTarget _at_ 2946;
int leftFootSideCurrent _at_ 2947;
int leftFootSidePwm _at_ 2948;
int leftFootSideVelo _at_ 2949;
int leftFootSideFlags _at_ 2951;
int leftFootSideAddonMix _at_ 2953;
//Правая кисть
int rightHandTarget _at_ 2962;
int rightHandCurrent _at_ 2963;
int rightHandPwm _at_ 2964;
int rightHandVelo _at_ 2965;
int rightHandFlags _at_ 2967;
int rightHandAddonMix _at_ 2969;
//Левая кисть
int leftHandTarget _at_ 2978;
int leftHandCurrent _at_ 2979;
int leftHandPwm _at_ 2980;
int leftHandVelo _at_ 2981;
int leftHandFlags _at_ 2983;
int leftHandAddonMix _at_ 2985;
//Головая наклон
int headTiltTarget _at_ 3010;
int headTiltCurrent _at_ 3011;
int headTiltPwm _at_ 3012;
int headTiltVelo _at_ 3013;
int headTiltFlags _at_ 3015;
int headTiltAddonMix _at_ 3017;
//Правое колено низ
int rightKneeBotTarget _at_ 3026;
int rightKneeBotCurrent _at_ 3027;
int rightKneeBotPwm _at_ 3028;
int rightKneeBotVelo _at_ 3029;
int rightKneeBotFlags _at_ 3031;
int rightKneeBotAddonMix _at_ 3033;
//Левое колено низ
int leftKneeBotTarget _at_ 3042;
int leftKneeBotCurrent _at_ 3043;
int leftKneeBotPwm _at_ 3044;
int leftKneeBotVelo _at_ 3045;
int leftKneeBotFlags _at_ 3047;
int leftKneeBotAddonMix _at_ 3049;
//Правая клешня
int rightClawTarget _at_ 3058;
int rightClawCurrent _at_ 3059;
int rightClawPwm _at_ 3060;
int rightClawVelo _at_ 3061;
int rightClawFlags _at_ 3063;
int rightClawAddonMix _at_ 3065;
//Левая клешня
int leftClawTarget _at_ 3074;
int leftClawCurrent _at_ 3075;
int leftClawPwm _at_ 3076;
int leftClawVelo _at_ 3077;
int leftClawFlags _at_ 3079;
int leftClawAddonMix _at_ 3081;


//Маски двигателей для групповых операций
#define MASK_HEAD_ROTATE 1 //Поворот головы
#define MASK_TORSO_ROTATE 2 //Поворот торса
#define MASK_RIGHT_CLAVICLE 4 //Правая ключица
#define MASK_LEFT_CLAVICLE 8 //Левая ключица
#define MASK_RIGHT_SHOULDER 16 //Правое плечо
#define MASK_LEFT_SHOULDER 32 //Левое плечо
#define MASK_RIGHT_ELBOW_SIDE 64 //Правый локоть вбок
#define MASK_LEFT_ELBOW_SIDE 128 //Левый локоть вбок
#define MASK_RIGHT_ELBOW 256 //Правый локоть вперед
#define MASK_LEFT_ELBOW 512 //Левый локоть вперед
#define MASK_RIGHT_PELVIC 1024 //Правый таз
#define MASK_LEFT_PELVIC 2048 //Левый таз
#define MASK_RIGHT_HIP_SIDE 4096 //Правое бедро вбок
#define MASK_LEFT_HIP_SIDE 8192 //Левое бедро вбок
#define MASK_RIGHT_HIP 16384 //Правое бедро
#define MASK_LEFT_HIP 32768 //Левое бедро
#define MASK_RIGHT_KNEE 65536 //Правое колено
#define MASK_LEFT_KNEE 131072 //Левое колено
#define MASK_RIGHT_FOOT_FRONT 262144 //Правая стопа вперед
#define MASK_LEFT_FOOT_FRONT 524288 //Левая стопа вперед
#define MASK_RIGHT_FOOT_SIDE 1048576 //Правая стопа вбок
#define MASK_LEFT_FOOT_SIDE 2097152 //Левая стопа вбок
#define MASK_RIGHT_HAND 4194304 //Правая кисть
#define MASK_LEFT_HAND 8388608 //Левая кисть
#define MASK_HEAD_TILT 33554432 //Головая наклон
#define MASK_RIGHT_KNEE_BOT 67108864 //Правое колено низ
#define MASK_LEFT_KNEE_BOT 134217728 //Левое колено низ
#define MASK_RIGHT_CLAW 268435456 //Правая клешня
#define MASK_LEFT_CLAW 536870912 //Левая клешня

#define MASK_ALL 1056964607 //Маска всех доступных моторов



//Индексы двигателей для индивидуальных операций
#define IDX_HEAD_ROTATE 0 //Поворот головы
#define IDX_TORSO_ROTATE 1 //Поворот торса
#define IDX_RIGHT_CLAVICLE 2 //Правая ключица
#define IDX_LEFT_CLAVICLE 3 //Левая ключица
#define IDX_RIGHT_SHOULDER 4 //Правое плечо
#define IDX_LEFT_SHOULDER 5 //Левое плечо
#define IDX_RIGHT_ELBOW_SIDE 6 //Правый локоть вбок
#define IDX_LEFT_ELBOW_SIDE 7 //Левый локоть вбок
#define IDX_RIGHT_ELBOW 8 //Правый локоть вперед
#define IDX_LEFT_ELBOW 9 //Левый локоть вперед
#define IDX_RIGHT_PELVIC 10 //Правый таз
#define IDX_LEFT_PELVIC 11 //Левый таз
#define IDX_RIGHT_HIP_SIDE 12 //Правое бедро вбок
#define IDX_LEFT_HIP_SIDE 13 //Левое бедро вбок
#define IDX_RIGHT_HIP 14 //Правое бедро
#define IDX_LEFT_HIP 15 //Левое бедро
#define IDX_RIGHT_KNEE 16 //Правое колено
#define IDX_LEFT_KNEE 17 //Левое колено
#define IDX_RIGHT_FOOT_FRONT 18 //Правая стопа вперед
#define IDX_LEFT_FOOT_FRONT 19 //Левая стопа вперед
#define IDX_RIGHT_FOOT_SIDE 20 //Правая стопа вбок
#define IDX_LEFT_FOOT_SIDE 21 //Левая стопа вбок
#define IDX_RIGHT_HAND 22 //Правая кисть
#define IDX_LEFT_HAND 23 //Левая кисть
#define IDX_HEAD_TILT 25 //Головая наклон
#define IDX_RIGHT_KNEE_BOT 26 //Правое колено низ
#define IDX_LEFT_KNEE_BOT 27 //Левое колено низ
#define IDX_RIGHT_CLAW 28 //Правая клешня
#define IDX_LEFT_CLAW 29 //Левая клешня



//Ссылки для прямого доступа к датчикам
// gaugeChannel0 - канал 0
// gaugeChannel1 - канал 1
// gaugeChannel2 - канал 2
//где gauge - имя датчика в таблице юнитов

#endif //ROKI2MET_H

