
//IronArt v0.103
//Данный файл сгенерирован автоматически системой IronArt и не должен редактироваться
//
#ifndef ROKI2MET_H
#define ROKI2MET_H
#include <Sv8Sys.h>

//Средние значения акселерометра
int svImuAccX         _at_ 1000;
int svImuAccY         _at_ 1001;
int svImuAccZ         _at_ 1002;
//Сырые значения акселерометра непосредственно с сенсора
int svImuAccRawX      _at_ 1003;
int svImuAccRawY      _at_ 1004;
int svImuAccRawZ      _at_ 1005;

//Средние значения гироскопа
int svImuGyroX        _at_ 1006;
int svImuGyroY        _at_ 1007;
int svImuGyroZ        _at_ 1008;
//Сырые значения гироскопа непосредственно с сенсора
int svImuGyroRawX     _at_ 1009;
int svImuGyroRawY     _at_ 1010;
int svImuGyroRawZ     _at_ 1011;

//Значения кватерниона
int svImuQuaterX      _at_ 1012;
int svImuQuaterY      _at_ 1013;
int svImuQuaterZ      _at_ 1014;
int svImuQuaterW      _at_ 1015;
int svImuQuaterAcc    _at_ 1016;

//Медленные значения акселерометра
int svImuAccSlowX     _at_ 1017;
int svImuAccSlowY     _at_ 1018;
int svImuAccSlowZ     _at_ 1019;


//Напряжение питания
int svAdc             _at_ 1020;

//Реакция на кнопки
//Запишите сюда номер слота, который должен выполняться при нажатии на кнопку
int svButtonLeft      _at_ 1021; //На левой кнопке по умолчанию расслабление
int svButtonRight     _at_ 1022; //На правой кнопке по умолчанию напряжение
int svButtonPress     _at_ 1023; //Состояние нажатых кнопок

//Маскирование управления движками от головы
int svHeadControlMask _at_ 1024;

//Углы Эйлера, получаемые после конвертации с помощью функции sfQuaternionToEuler
float svEulerYaw      _at_ 1025;
float svEulerPitch    _at_ 1026;
float svEulerRoll     _at_ 1027;


#define SV_BUTTON_LEFT_PRESS  1 //Битовая маска для левой кнопки
#define SV_BUTTON_RIGHT_PRESS 2 //Битовая маска для правой кнопки




//Управление от внешнего пульта
int svRemoteStatus         _at_ 1190;  //Состояние пульта (подключен или нет)
int svRemoteButton         _at_ 1191;  //Состояние кнопок пульта (битовое поле, 0 - не нажата, 1 - нажата)
int svRemoteLeftJoystick   _at_ 1192;  //Кнопка левого джойстика
int svRemoteLeftJoystickX  _at_ 1193;  //Смещение левого джойстика по X
int svRemoteLeftJoystickY  _at_ 1194;  //Смещение левого джойстика по Y
int svRemoteRightJoystick  _at_ 1195;  //Кнопка правого джойстика
int svRemoteRightJoystickX _at_ 1196;  //Смещение правого джойстика по X
int svRemoteRightJoystickY _at_ 1197;  //Смещение правого джойстика по Y

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



//Индексы слотов, которые можно применять в svButtonLeft, svButtonRight
#define SV_SLOT_MAX            64 //Максимальное количество слотов для скриптов
#define SV_SLOT_STOP           65 //Остановить текущий скрипт
#define SV_SLOT_RESTART_RUN    66 //Перезапустить текущий скрипт и сразу стартовать
#define SV_SLOT_RESTART_PAUSE  67 //Перезапустить текущий скрипт в режиме паузы
#define SV_SLOT_ZERO_POS       68 //Перейти в нулевую позицию
#define SV_SLOT_RELAX          69 //Расслабить моторы
#define SV_SLOT_INACTIVE       -1 //Нет операции со слотом




//Модель гуманоидного робота для вычисления прямой и обратной кинематики
int svHumoModelDistPelvic       _at_ 1204;  //Расстояние от центра до точки поворота таза
int svHumoModelLengthPelvicHip  _at_ 1205;  //Расстояние от точки поворота таза до поворота бедра вбок
int svHumoModelLengthHipHip     _at_ 1206;  //Расстояние между осями поворота бедра вбок и вперед
int svHumoModelLengthHipKnee    _at_ 1207;  //Длина бедра до колена
int svHumoModelLengthKneeFoot   _at_ 1208;  //Длина голени от колена до поворота стопы вперед
int svHumoModelLengthFootFoot   _at_ 1209;  //Расстояние между осями поворота стопы вбок и вперед
int svHumoModelDistFootFloor    _at_ 1210;  //Расстояние между осью поворота стопы вбок и землей
int svHumoModelFootLength       _at_ 1211;  //Длина стопы
int svHumoModelFootWidth        _at_ 1212;  //Ширина стопы
int svHumoModelFootCenterBack   _at_ 1213; //Расстояние от задника стопы до оси поворота
int svHumoModelFootCenterInner  _at_ 1214; //Расстояние от внутренней части стопы до оси поворота

int svHumoModelAngPelvic        _at_ 1198; //Угол поворота таза
int svHumoModelAngHipSide       _at_ 1200; //Угол поворота бедра вбок
int svHumoModelAngHip           _at_ 1199; //Угол поворота бедра вперед
int svHumoModelAngKnee          _at_ 1201; //Угол поворота колена
int svHumoModelAngFoot          _at_ 1202; //Угол поворота ступни вперед
int svHumoModelAngFootSide      _at_ 1203; //Угол поворота ступни вбок



//Система вычисления обратной кинематики по методу Азера
float svIkA5   _at_ 1215;  // мм расстояние от оси симметрии до оси сервы 5
float svIkB5   _at_ 1216;  // мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
float svIkC5   _at_ 1217;  // мм расстояние от оси сервы 6 до нуля Z по вертикали
float svIkA6   _at_ 1218;  // мм расстояние от оси сервы 6 до оси сервы 7
float svIkA7   _at_ 1219;  // мм расстояние от оси сервы 7 до оси сервы 8
float svIkA8   _at_ 1220;  // мм расстояние от оси сервы 8 до оси сервы 9
float svIkA9   _at_ 1221;  // мм расстояние от оси сервы 9 до оси сервы 10
float svIkA10  _at_ 1222;  // мм расстояние от оси сервы 10 до центра стопы по горизонтали
float svIkB10  _at_ 1223;  // мм расстояние от оси сервы 10 до низа стопы
float svIkC10  _at_ 1224; // мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали

//Лимиты вращения двигателей в радианах
float svIkLimA5min  _at_ 1225;
float svIkLimA5max  _at_ 1226;
float svIkLimA6min  _at_ 1227;
float svIkLimA6max  _at_ 1228;
float svIkLimA7min  _at_ 1229;
float svIkLimA7max  _at_ 1230;
float svIkLimA8min  _at_ 1231;
float svIkLimA8max  _at_ 1232;
float svIkLimA9min  _at_ 1233;
float svIkLimA9max  _at_ 1234;
float svIkLimA10min _at_ 1235;
float svIkLimA10max _at_ 1236;

int svIkOutPresent  _at_ 1237; //Флаг наличия результатов вычисления, 0 - когда нету решения

int svIkEncA5       _at_ 1238; //Угол поворота таза
int svIkEncA6       _at_ 1239; //Угол поворота бедра вбок
int svIkEncA7       _at_ 1240; //Угол поворота бедра вперед
int svIkEncA8       _at_ 1241; //Угол поворота колена
int svIkEncA9       _at_ 1242; //Угол поворота ступни вперед
int svIkEncA10      _at_ 1243; //Угол поворота ступни вбок


//Ссылки для прямого доступа к двигателям
// motorTarget   - целевой угол двигателя
// motorCurrent  - текущий фактический угол поворота выходного вала двигателя
// motorPwm      - текущий ШИМ, подаваемый на двигатель
// motorAddonMix - добавочная величина, которая добавляется при к целевому углу двигателя
//где motor - имя двигателя в таблице юнитов
//Поворот головы
int headRotateTarget _at_ 1030;
int headRotateCurrent _at_ 1031;
int headRotatePwm _at_ 1032;
int headRotateAddonMix _at_ 1033;
//Поворот торса
int torsoRotateTarget _at_ 1034;
int torsoRotateCurrent _at_ 1035;
int torsoRotatePwm _at_ 1036;
int torsoRotateAddonMix _at_ 1037;
//Правая ключица
int rightClavicleTarget _at_ 1038;
int rightClavicleCurrent _at_ 1039;
int rightClaviclePwm _at_ 1040;
int rightClavicleAddonMix _at_ 1041;
//Левая ключица
int leftClavicleTarget _at_ 1042;
int leftClavicleCurrent _at_ 1043;
int leftClaviclePwm _at_ 1044;
int leftClavicleAddonMix _at_ 1045;
//Правое плечо
int rightShoulderTarget _at_ 1046;
int rightShoulderCurrent _at_ 1047;
int rightShoulderPwm _at_ 1048;
int rightShoulderAddonMix _at_ 1049;
//Левое плечо
int leftShoulderTarget _at_ 1050;
int leftShoulderCurrent _at_ 1051;
int leftShoulderPwm _at_ 1052;
int leftShoulderAddonMix _at_ 1053;
//Правый локоть вбок
int rightElbowSideTarget _at_ 1054;
int rightElbowSideCurrent _at_ 1055;
int rightElbowSidePwm _at_ 1056;
int rightElbowSideAddonMix _at_ 1057;
//Левый локоть вбок
int leftElbowSideTarget _at_ 1058;
int leftElbowSideCurrent _at_ 1059;
int leftElbowSidePwm _at_ 1060;
int leftElbowSideAddonMix _at_ 1061;
//Правый локоть вперед
int rightElbowTarget _at_ 1062;
int rightElbowCurrent _at_ 1063;
int rightElbowPwm _at_ 1064;
int rightElbowAddonMix _at_ 1065;
//Левый локоть вперед
int leftElbowTarget _at_ 1066;
int leftElbowCurrent _at_ 1067;
int leftElbowPwm _at_ 1068;
int leftElbowAddonMix _at_ 1069;
//Правый таз
int rightPelvicTarget _at_ 1070;
int rightPelvicCurrent _at_ 1071;
int rightPelvicPwm _at_ 1072;
int rightPelvicAddonMix _at_ 1073;
//Левый таз
int leftPelvicTarget _at_ 1074;
int leftPelvicCurrent _at_ 1075;
int leftPelvicPwm _at_ 1076;
int leftPelvicAddonMix _at_ 1077;
//Правое бедро вбок
int rightHipSideTarget _at_ 1078;
int rightHipSideCurrent _at_ 1079;
int rightHipSidePwm _at_ 1080;
int rightHipSideAddonMix _at_ 1081;
//Левое бедро вбок
int leftHipSideTarget _at_ 1082;
int leftHipSideCurrent _at_ 1083;
int leftHipSidePwm _at_ 1084;
int leftHipSideAddonMix _at_ 1085;
//Правое бедро
int rightHipTarget _at_ 1086;
int rightHipCurrent _at_ 1087;
int rightHipPwm _at_ 1088;
int rightHipAddonMix _at_ 1089;
//Левое бедро
int leftHipTarget _at_ 1090;
int leftHipCurrent _at_ 1091;
int leftHipPwm _at_ 1092;
int leftHipAddonMix _at_ 1093;
//Правое колено
int rightKneeTarget _at_ 1094;
int rightKneeCurrent _at_ 1095;
int rightKneePwm _at_ 1096;
int rightKneeAddonMix _at_ 1097;
//Левое колено
int leftKneeTarget _at_ 1098;
int leftKneeCurrent _at_ 1099;
int leftKneePwm _at_ 1100;
int leftKneeAddonMix _at_ 1101;
//Правая стопа вперед
int rightFootFrontTarget _at_ 1102;
int rightFootFrontCurrent _at_ 1103;
int rightFootFrontPwm _at_ 1104;
int rightFootFrontAddonMix _at_ 1105;
//Левая стопа вперед
int leftFootFrontTarget _at_ 1106;
int leftFootFrontCurrent _at_ 1107;
int leftFootFrontPwm _at_ 1108;
int leftFootFrontAddonMix _at_ 1109;
//Правая стопа вбок
int rightFootSideTarget _at_ 1110;
int rightFootSideCurrent _at_ 1111;
int rightFootSidePwm _at_ 1112;
int rightFootSideAddonMix _at_ 1113;
//Левая стопа вбок
int leftFootSideTarget _at_ 1114;
int leftFootSideCurrent _at_ 1115;
int leftFootSidePwm _at_ 1116;
int leftFootSideAddonMix _at_ 1117;
//Правая кисть
int rightHandTarget _at_ 1118;
int rightHandCurrent _at_ 1119;
int rightHandPwm _at_ 1120;
int rightHandAddonMix _at_ 1121;
//Левая кисть
int leftHandTarget _at_ 1122;
int leftHandCurrent _at_ 1123;
int leftHandPwm _at_ 1124;
int leftHandAddonMix _at_ 1125;
//Головая наклон
int headTiltTarget _at_ 1130;
int headTiltCurrent _at_ 1131;
int headTiltPwm _at_ 1132;
int headTiltAddonMix _at_ 1133;
//Правое колено низ
int rightKneeBotTarget _at_ 1134;
int rightKneeBotCurrent _at_ 1135;
int rightKneeBotPwm _at_ 1136;
int rightKneeBotAddonMix _at_ 1137;
//Левое колено низ
int leftKneeBotTarget _at_ 1138;
int leftKneeBotCurrent _at_ 1139;
int leftKneeBotPwm _at_ 1140;
int leftKneeBotAddonMix _at_ 1141;
//Правая клешня
int rightClawTarget _at_ 1142;
int rightClawCurrent _at_ 1143;
int rightClawPwm _at_ 1144;
int rightClawAddonMix _at_ 1145;
//Левая клешня
int leftClawTarget _at_ 1146;
int leftClawCurrent _at_ 1147;
int leftClawPwm _at_ 1148;
int leftClawAddonMix _at_ 1149;


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

