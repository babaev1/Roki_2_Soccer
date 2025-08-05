/*
  Проект "Roki-2"
  Автор
    Зубр ОВК
  Описание
    Скрипт обеспечивает переназначение правой кнопки и переводит все сервы в 0
*/
#include <roki2met.h>

void main() {
  //Переназначаем кнопку на нужный слот
  //svButtonRight = sfSlotIndex("roki2Walk_test.cpp");
  svButtonRight = sfSlotIndex("roki2Walk_Remote.cpp");
  //svButtonRight = sfSlotIndex("roki4Jump.cpp");
  //Переводим все сервы в 0
  sfPoseGroup( MASK_ALL, 0, 100 );
  sfWaitFrame( 100 );
  svHeadControlMask = -1;       //accept servo signals from head
  //sfSoundPlay(1);
  //sfStop();
  sfStartSlot( sfSlotIndex("mixingRoki2met.cpp") );
  }
