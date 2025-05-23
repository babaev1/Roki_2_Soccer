/*
  Проект "Металлический робот"
  Автор
    Зубр ОВК
  Описание
    Скрипт, с помощью которого осуществляется миксирование на ступни значений с IMU

    Для добавления миксования к другому скрипту функцию mixing можно вызывать в параллельном потоке, например
    svCreateTask( mixing, 50 );
*/
#include <roki2met.h>
#include <roki2global.h>

//Факторы определяют степень зависимости поворота стоп от показаний IMU
int leftRightFactor; //Фактор миксования лево-право
int frontBackFactor; //Фактор миксования вперед-назад

int quart;
int errCount;
float sum;
int right_knee_ntc;
int left_knee_ntc;
int right_knee_bot_ntc;
int left_knee_bot_ntc;

void paramScan() {
  while(1) {
    sfUnitParamRead( IDX_RIGHT_KNEE, 201, &right_knee_ntc );
    sfUnitParamRead( IDX_LEFT_KNEE, 201, &left_knee_ntc );
    sfUnitParamRead( IDX_RIGHT_KNEE_BOT, 201, &right_knee_bot_ntc );
    sfUnitParamRead( IDX_LEFT_KNEE_BOT, 201, &left_knee_bot_ntc );
    lowest_ntc = right_knee_ntc;
    if ( lowest_ntc > left_knee_ntc) lowest_ntc = left_knee_ntc;
    if ( lowest_ntc > right_knee_bot_ntc) lowest_ntc = right_knee_bot_ntc;
    if ( lowest_ntc > left_knee_bot_ntc) lowest_ntc = left_knee_bot_ntc;
    }
}


void mixing() {
  //Начальные установки
  leftRightFactor = 100;
  frontBackFactor = 100;

  while(1) {

    //По боковому крену
    leftFootSideAddonMix = -svImuGyroZ * leftRightFactor >> 10;
    rightFootSideAddonMix = svImuGyroZ * leftRightFactor >> 10;
    
    //По крену вперед-назад
    rightFootFrontAddonMix = leftFootFrontAddonMix = -svImuGyroX * frontBackFactor >> 10;

    //Эта вставка контролирует правильность показаний кватерниона
    //Здесь мы считаем сумму квадратов
    quart = svImuQuaterX * svImuQuaterX + svImuQuaterY * svImuQuaterY + svImuQuaterZ * svImuQuaterZ + svImuQuaterW * svImuQuaterW;
    //Здесь извлекается квадратный корень
    sum = sfFMathSqrt( quart );
    //Здесь сравниваем с единицей
    if( sfAbs(sum - 16384.0) > 164 )
      //Увеличиваем счетчик ошибок на 1 если результат отличается от единицы больше, чем на 0.01
      errCount++;
        
    //Ожидаем до следующего фрейма
    sfWaitNextFrame();
    }
  }

void main() {
  sfCreateTask( paramScan, 20 );
  mixing();
  }
