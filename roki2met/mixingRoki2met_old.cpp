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

//Факторы определяют степень зависимости поворота стоп от показаний IMU
int leftRightFactor; //Фактор миксования лево-право
int frontBackFactor; //Фактор миксования вперед-назад


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
    
    //Ожидаем до следующего фрейма
    sfWaitNextFrame();
    }
  }

void main() {
  mixing();
  }
