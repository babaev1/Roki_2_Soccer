/*
  Проект "Roki-2"
  Автор
    Зубр ОВК
  Описание
    Скрипт прошивки серийного номера
   Замените аргумент sfSerialSet на нужный номер и запустите скрипт.
*/
#include <roki2met.h>

import VPS_SERIAL_SET   void sfSerialSet( int serial );
import VPS_SERIAL_GET   int  sfSerialGet();

int s;

void main() {
  s = sfSerialGet();
  sfSerialSet(288);
  sfStop();
  }


