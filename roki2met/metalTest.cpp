/*
  Проект "Металлический робот"
  Автор
    Зубр ОВК
  Описание
    Скрипт проверки всякой хрени
*/
#include <roki2met.h>

void main() {
  sfPoseGroup(MASK_LEFT_CLAW, 0, 100);
  sfPoseGroup(MASK_RIGHT_CLAW, 0, 100);
  sfStop();
}