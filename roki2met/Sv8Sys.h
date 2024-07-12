/*
  Проект     "Скриптовый язык SaliScript: упрощенный c++"
  Подпроект  "Виртуальная машина"
  Автор
    Alexander Sibilev
  Интернет
    www.rc.saliLab.ru - домашний сайт проекта
    www.saliLab.ru
    www.saliLab.com

  Описание
    Стандартные внешние функции-драйверы, обработка которых
    реализована в классе SrVMachine
*/
#ifndef SV8SYS_H
#define SV8SYS_H

#define VPU_VERSION8                  8

#define VPS_VERSION                   1  //Получить версию VPU
#define VPS_CREATETASK                2  //Создать новую задачу (активировать следующий VPU)

#define VPS_SERIAL_SET                10 //Установить новый серийный номер bluetooth адаптера
#define VPS_SERIAL_GET                11 //Получить серийный номер bluetooth адаптера

#define VPS_LOG                       14 //Записать log - файл
#define VPS_LOG_OPEN                  15 //Открыть log-файл (организовать в виде стека)
#define VPS_LOG_CLOSE                 16 //Закрыть log-файл

#define VPS_WAIT_FRAME_OUT            17 //Ожидать пока время не превысит заданное
#define VPS_IS_FRAME_OUT              18 //Проверить, истекло ли заданное время

//Математика
#define VPS_ABS                       20 //Абсолютное значение числа
#define VPS_IMIN                      21 //Получение минимального из двух значений
#define VPS_IMAX                      22 //Получение максимального из двух значений
#define VPS_UMIN                      23 //Получение минимального из двух значений для беззнакового числа
#define VPS_UMAX                      24 //Получение максимального из двух значений для беззнакового числа
#define VPS_ILIMIT                    25 //Ограничение целого числа

//Прочие функции
#define VPS_RND                       40 //Получить случайное число в заданном диапазоне
#define VPS_STOP                      41 //Останов скрипта
#define VPS_START_SLOT                42 //Запустить на выполнение заданный слот
#define VPS_BIP                       43 //Звуковой сигнал

#define VPS_FREE_GROUP                50 //Отключить управление движками
#define VPS_POSE_GROUP                51 //Задание новой позиции группы механизмов
#define VPS_POSE_GROUP_DELAY          52 //Задание новой позиции группы механизмов с задержкой
#define VPS_POSE_CORRECT              53 //Корректировка целевой позиции группы механизмов
#define VPS_QUEUE_GROUP               54 //Добавить в очередь
#define VPS_POSE_IDX                  55 //Задание новой позиции для отдельного механизма
#define VPS_POSE_GROUP_LIN            56 //Задание новой позиции группы механизмов с линейной интерполяцией
#define VPS_STRAIN_GROUP              57 //Напрячь движки (держать текущую позицию)
#define VPS_PLAY_SAMPLES              58 //Проиграть последовательность относительных выборок

#define VPS_ACCUM_SETUP_1             60 //Настроить аккумулятор
#define VPS_ACCUM_SETUP_2             61 //Настроить аккумулятор
#define VPS_ACCUM_SETUP_3             62 //Настроить аккумулятор
#define VPS_ACCUM_SETUP_4             63 //Настроить аккумулятор
#define VPS_ACCUM_TRIGGER             64 //Запустить аккумулятор, повторный запуск не возможен пока не будет сделан RESET или SETUP
#define VPS_ACCUM_RESET               65 //Сброс аккумулятора, после него возможен запуск

#define VPS_MATH_ACOS                 80 //Возвращает acos отношения в попугаях энкодера
#define VPS_MATH_SQRTI                81 //Возвращает квадратный корень для значений от 0 до 300
#define VPS_MATH_ANG_OF_TRI           82 //Возвращает угол по трем сторонам треугольника
#define VPS_MATH_ANG_OF_KNEE          83 //Возвращает угол колена по расстоянию от бедра до щиколотки
#define VPS_CATHET_OF_SQUARE_TRI      84 //Возвращает неизвестный катет прямоугольного треугольника
#define VPS_HYPO_OF_SQUARE_TRI        85 //Возвращает гипотенузу прямоугольного треугольника по двум катетам
#define VPS_TABLE_SIN                 86 //Преобразование значения по синусоидальному закону

#define VPS_HUMO_MODEL_FOOT_AT_FLOOR 100 //Выполняет расчет положений серв для ступни параллельной полу
#define VPS_HUMO_MODEL_SETUP         101 //Настроить модель
#define VPS_HUMO_MODEL_FOOT_PUSH     102 //Выполняет расчет положений серв для толкания ступней

#define VPS_IDX_GROUP                150 //Возвращает индекс первого элемента в группе
#define VPS_QUEUE_IDX_APPEND         151 //Добавить в очередь одно значение
#define VPS_QUEUE_IDX_OFFSETS        152 //Добавить в очередь таблицу смещений
#define VPS_QUEUE_IDX_SIN_MOVE       153 //Добавить в очередь синусоидальное движение
#define VPS_QUEUE_IDX_LAST           154 //Возвращает последнее значение в очереди
#define VPS_QUEUE_IDX_COUNT          155 //Возвращает количество элементов в очереди
#define VPS_QUEUE_MAX                156 //Максимальное количество элементов в очереди

#define VPS_AZER_IK_SETUP            200 //Инициализировать вычислитель обратной кинематики Азера
#define VPS_AZER_IK_ANGLE            201 //Вычислить обратную кинематику методом Азера

#define VPS_MATH_COS                 300
#define VPS_MATH_SIN                 301
#define VPS_QUATERNION_TO_EULER      302
#define VPS_QUATERNION_TO_EULER_IMU  303

#define VPS_FMATH_ACOS               351 //Вычисляет арккосинус в диапазоне -PI до +PI для аргумента в диапазоне -1 до +1
#define VPS_FMATH_ASIN               352 //Вычисляет арксинус в диапазоне -PI до +PI для аргумента в диапазоне -1 до +1
#define VPS_FMATH_ATAN               353 //Вычисляет основной арктангенс в диапазоне -PI/2 до +PI/2
#define VPS_FMATH_ATAN2              354 //Вычисляет арктангенс отношения y/x, используя знаки аргументов, чтобы определить правильный квадрант.
#define VPS_FMATH_COS                355 //Вычисляет косинус угла в радианах
#define VPS_FMATH_SIN                356 //Вычисляет синус угла в радианах
#define VPS_FMATH_TAN                357 //Вычисляет тангенс угла в радианах
#define VPS_FMATH_COSH               358 //Вычисляет гиперболический косинус
#define VPS_FMATH_SINH               359 //Вычисляет гиперболический косинус
#define VPS_FMATH_TANH               360 //Вычисляет гиперболический тангенс
#define VPS_FMATH_EXP                361 //Вычисляет e (число Эйлера, 2.7182818...), возведенное в заданную степень
#define VPS_FMATH_FREXP              362 //Разлагает заданное значение с плавающей запятой num на нормализованную дробь и целую степень двойки.
#define VPS_FMATH_LDEXP              363 //Умножает значение с плавающей запятой num на число 2, возведенное в степень exp.
#define VPS_FMATH_LOG                364 //Вычисляет натуральный (по основанию e) логарифм числа
#define VPS_FMATH_LOG10              365 //Вычисляет общий (по основанию 10) логарифм числа
#define VPS_FMATH_MODF               366 //Разлагает заданное значение с плавающей запятой num на целую и дробную части, каждая из которых имеет
                                         // тот же тип и знак, что и num. Целая часть (в формате с плавающей запятой) хранится в объекте, на
                                         // который указывает ptr.
#define VPS_FMATH_POW                367 //Вычисляет значение базы, возведенное в степень exp.
#define VPS_FMATH_SQRT               368 //Вычисляет квадратный корень числа
#define VPS_FMATH_CEIL               369 //Вычисляет наименьшее целое значение, не меньшее num. (Округление в большую сторону)
#define VPS_FMATH_FABS               370 //Вычисляет абсолютное значение числа
#define VPS_FMATH_FLOOR              371 //Вычисляет наибольшее целочисленное значение, не превышающее num. (Округление в меньшую сторону)
#define VPS_FMATH_FMOD               372 //Вычисляет остаток с плавающей запятой от операции деления x / y.



#define MATH_PI                3.141592654
#define MATH_RAD_PER_DEGREE    0.017453293

#define ENC_PER_DEGREE         45.511111 //Преобразование из градусов в значения энкодера
#define ENC_PER_RADIAN       2607.594587 //Преобразование из радиан в значения энкодера


#ifdef VPU_COMPILER
//Обязательная переменная отражающая счетчик фреймов
int svFrameCount _at_ 1;

//Возвращает версию vpu
import VPS_VERSION          int  sfVpuVersion();

//Создание нового виртуального процессора, который будет работать параллельно
import VPS_CREATETASK       int  sfCreateTask( void (*fun)(), int stackSize );

//import VPS_SERIAL_GET       int  sfSerialGet();

import VPS_LOG              void sfLog( cstring title, int p1, int p2, int p3 );
import VPS_LOG_OPEN         void sfLogOpen( cstring fname );
import VPS_LOG_CLOSE        void sfLogClose();

//Ожидать пока не наступит заданный фрейм
import VPS_WAIT_FRAME_OUT   void sfWaitFrameOut( int frame );

//Проверяет, наступил ли заданный фрейм
import VPS_IS_FRAME_OUT     int  sfIsFrameOut( int frame );

//Ожидает до следующего фрейма
                            void sfWaitNextFrame();

//Возвращает индекс слота по его имени
                            int  sfSlotIndex( cstring str );

//Возвращает абсолютное значение числа
import VPS_ABS              int  sfAbs( int val );
//Возвращает минимальное из двух чисел
import VPS_IMIN             int  sfIMin( int v1, int v2 );
//Возвращает максимальное из двух чисел
import VPS_IMAX             int  sfIMax( int v1, int v2 );
//Возвращает минимальное из двух беззнаковых чисел
import VPS_UMIN             int  sfUMin( int v1, int v2 );
//Возвращает максимальное из двух беззнаковых чисел
import VPS_UMAX             int  sfUMax( int v1, int v2 );
//Возвращает значение, ограниченное минимальным и максимальным
import VPS_ILIMIT           int  sfILimit( int val, int minVal, int maxVal );

//Останавливает исполнение скрипта
import VPS_STOP             void sfStop();
//Запускает скрипт из заданного слота
import VPS_START_SLOT       void sfStartSlot( int slotIndex );
//Формирует звуковой сигнал
import VPS_BIP              void sfBip( int bipCount, int seriesCount );


//Отключает управление для группы двигателей
import VPS_FREE_GROUP       void sfFreeGroup( int unitMask );
//Запускает позиционирование группы на заданный угол за количество фреймов
import VPS_POSE_GROUP       void sfPoseGroup( int unitMask, int angle, int frameCount );
//Запускает позиционирование группы на заданный угол за количество фреймов с задержкой исполнения
import VPS_POSE_GROUP_DELAY void sfPoseGroupDelay( int unitMask, int angle, int frameCount, int frameDelay );
//Корректирует конечную точку позиционирования для группы устройств
import VPS_POSE_CORRECT     void sfPoseCorrect( int unitMask, int angle );
//Добавляет задание в очередь исполнения
import VPS_QUEUE_GROUP      void sfQueueGroup( int unitMask, int angle, int frameCount );
//Запускает позиционирование индивидуального механизма
import VPS_POSE_IDX         void sfPoseIdx( int idx, int angle, int frameCount );
//Задание новой позиции группы механизмов с линейной интерполяцией
import VPS_POSE_GROUP_LIN   void sfPoseGroupLin( int unitMask, int angle, int frameCount );
//Напрячь группу двигателей
import VPS_STRAIN_GROUP     void sfStrainGroup( int unitMask );

import VPS_PLAY_SAMPLES     void sfPlaySamples( int unitMask, cblock *samples, int frameCount );

//Настраивает аккумулятор для одной переменной
import VPS_ACCUM_SETUP_1    void sfAccumSetup1( int *ch0 );
//Настраивает аккумулятор для двух переменных
import VPS_ACCUM_SETUP_2    void sfAccumSetup2( int *ch0, int *ch1 );
//Настраивает аккумулятор для трех переменных
import VPS_ACCUM_SETUP_3    void sfAccumSetup3( int *ch0, int *ch1, int *ch2 );
//Настраивает аккумулятор для четырех переменных
import VPS_ACCUM_SETUP_4    void sfAccumSetup4( int *ch0, int *ch1, int *ch2, int *ch3 );
//Запускает аккумулирование
import VPS_ACCUM_TRIGGER    void sfAccumTrigger();
//Сбрасывает аккумулятор
import VPS_ACCUM_RESET      void sfAccumReset();



//Возвращает acos отношения в попугаях энкодера
import VPS_MATH_ACOS                int  sfMathAcos( int numerator, int denumerator );
//Возвращает квадратный корень для значений от 0 до 300
import VPS_MATH_SQRTI               int  sfMathSqrti( int v );
//Возвращает угол по трем сторонам треугольника
import VPS_MATH_ANG_OF_TRI          int  sfMathAngleOfTriangle( int sideA, int sideB, int sideC );
//Возвращает угол колена по расстоянию от бедра до щиколотки
import VPS_MATH_ANG_OF_KNEE         int  sfMathAngleOfKnee( int hip, int shin, int height );
//Возвращает неизвестный катет прямоугольного треугольника
import VPS_CATHET_OF_SQUARE_TRI     int  sfMathCathetOfSquareTriangle( int cathet, int hypotenuse );
//Возвращает гипотенузу прямоугольного треугольника по двум катетам
import VPS_HYPO_OF_SQUARE_TRI       int  sfMathHypotenuseOfSquareTriangle( int cathet1, int cathet2 );
//Преобразование значения по синусоидальному закону
import VPS_TABLE_SIN                int  sfMathTableSin( int index, int value );

//Выполняет настройку модели гуманоида
import VPS_HUMO_MODEL_SETUP         void sfHumoModelSetup();
//Выполняет расчет положений серв для ступни параллельной полу
import VPS_HUMO_MODEL_FOOT_AT_FLOOR void sfHumoModelFootAtFloor( int front, int side, int height );
//Выполняет расчет положений серв для толчка стопой назад
import VPS_HUMO_MODEL_FOOT_PUSH     void sfHumoModelFootPush( int back, int side, int height );


//Возвращает индекс первого элемента в группе
import VPS_IDX_GROUP                int  sfIndexOfMask( int unitMask );

//Разместить в очереди сервы значение
import VPS_QUEUE_IDX_APPEND         int  sfQueueIdxAppend( int unitIndex, int val, int frameCount );

//Разместить в очереди таблицу смещений
import VPS_QUEUE_IDX_OFFSETS        int  sfQueueIdxOffsets( int unitIndex, int from, cblock *offsets, int offsetCount );

//Разместить в очереди синусоидальное движение
import VPS_QUEUE_IDX_SIN_MOVE       int  sfQueueIdxSinMove( int unitIndex, int from, int to, int frameCount );

//Возвращает последнее значение в очереди
import VPS_QUEUE_IDX_LAST           int  sfQueueIdxLast( int unitIndex );

//Возвращает количество элементов в очереди
import VPS_QUEUE_IDX_COUNT          int  sfQueueIdxCount( int unitIndex );

//Максимальное количество элементов в очереди
import VPS_QUEUE_MAX                int  sfQueueMax();


//Настройка модели робота для инверсной кинематики по методу Азера
import VPS_AZER_IK_SETUP            void sfIkSetup();

//Вычисление углов движков методом Азера
import VPS_AZER_IK_ANGLE            void sfIkAngle( float xt, float yt, float zt, float x, float y, float z, float w );

//Возвращает значение косинуса угла, угол в радианах
import VPS_MATH_COS                 float sfMathCos( float r );

//Возвращает значение синуса угла, угол в радианах
import VPS_MATH_SIN                 float sfMathSin( float r );

//Переводит кватернион в углы эйлера
import VPS_QUATERNION_TO_EULER      void sfQuaternionToEuler( float x, float y, float z, float w );

//Переводит кватернион из IMU в углы эйлера
import VPS_QUATERNION_TO_EULER_IMU  void sfQuaternionToEulerImu();




//Вычисляет арккосинус в диапазоне -PI до +PI для аргумента в диапазоне -1 до +1
import VPS_FMATH_ACOS               float sfFMathACos( float v );

//Вычисляет арксинус в диапазоне -PI до +PI для аргумента в диапазоне -1 до +1
import VPS_FMATH_ASIN               float sfFMathASin( float v );

//Вычисляет основной арктангенс в диапазоне -PI/2 до +PI/2
import VPS_FMATH_ATAN               float sfFMathATan( float v );

//Вычисляет арктангенс отношения y/x, используя знаки аргументов, чтобы определить правильный квадрант.
import VPS_FMATH_ATAN2              float sfFMathATan2( float y, float x );

//Возвращает значение косинуса угла, угол в радианах
import VPS_FMATH_COS                float sfFMathCos( float r );

//Возвращает значение синуса угла, угол в радианах
import VPS_FMATH_SIN                float sfFMathSin( float r );

//Вычисляет тангенс угла в радианах
import VPS_FMATH_TAN                float sfFMathTan( float r );

//Вычисляет гиперболический косинус
import VPS_FMATH_COSH               float sfFMathCosh( float v );

//Вычисляет гиперболический косинус
import VPS_FMATH_SINH               float sfFMathSinh( float v );

//Вычисляет гиперболический тангенс
import VPS_FMATH_TANH               float sfFMathTanh( float v );

//Вычисляет e (число Эйлера, 2.7182818...), возведенное в заданную степень
import VPS_FMATH_EXP                float sfFMathExp( float v );

//Разлагает заданное значение с плавающей запятой num на нормализованную дробь и целую степень двойки.
import VPS_FMATH_FREXP              float sfFMathFrExp( float v, int *num );

//Умножает значение с плавающей запятой num на число 2, возведенное в степень exp.
import VPS_FMATH_LDEXP              float sfFMathLdExp( float num, int exp );

//Вычисляет натуральный (по основанию e) логарифм числа
import VPS_FMATH_LOG                float sfFMathLog( float v );

//Вычисляет общий (по основанию 10) логарифм числа
import VPS_FMATH_LOG10              float sfFMathLog10( float v );

//Разлагает заданное значение с плавающей запятой num на целую и дробную части, каждая из которых имеет
// тот же тип и знак, что и num. Целая часть (в формате с плавающей запятой) хранится в объекте, на который указывает ptr.
import VPS_FMATH_MODF               float sfFMathModf( float num, float *ptr );

//Вычисляет значение базы, возведенное в степень exp.
import VPS_FMATH_POW                float sfFMathPow( float base, float exp );

//Вычисляет квадратный корень числа
import VPS_FMATH_SQRT               float sfFMathSqrt( float v );

//Вычисляет наименьшее целое значение, не меньшее num. (Округление в большую сторону)
import VPS_FMATH_CEIL               float sfFMathCeil( float v );

//Вычисляет абсолютное значение числа
import VPS_FMATH_FABS               float sfFMathFAbs( float v );

//Вычисляет наибольшее целочисленное значение, не превышающее num. (Округление в меньшую сторону)
import VPS_FMATH_FLOOR              float sfFMathFloor( float v );

//Вычисляет остаток с плавающей запятой от операции деления x / y.
import VPS_FMATH_FMOD               float sfFMathFMod( float x, float y );





//Ожидать пока не пройдет заданное количество фреймов
void sfWaitFrame( int frame ) {
  sfWaitFrameOut( frame + svFrameCount );
  }


#endif


#endif // SV8SYS_H
