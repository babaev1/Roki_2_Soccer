/*
  Проект "Roki-2"
  Автор
    Зубр ОВК
  Описание
    Скрипт обеспечивает переназначение кнопок.
    Левая кнопка снова запускает этот скрипт, который переназначает кнопки и одновременно расслабляет все двигатели
*/
#include <roki2met.h>
#include <roki2global.h>

import VPS_SERIAL_GET   int  sfSerialGet();


void main() {
  robot_Serial_Number = sfSerialGet();
  splits_Mode = 0;
  //На правую кнопку назначаем "Напрячся", на левую - этот скрипт, в котором расслабляемся
  svButtonRight = sfSlotIndex("roki2TenseUp.cpp");
  svButtonLeft  = sfSlotIndex("roki2Init.cpp");
  //Расслабиться
  sfFreeGroup( MASK_ALL );
  svHeadControlMask = 0;       // ignore servo signals from head
  //... и завершить
  sfStop();
  }
