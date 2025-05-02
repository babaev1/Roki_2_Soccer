/*
  Проект "Роки-2"
  Автор
    Зубр ОВК
  Описание
    Скрипт микропрыжки
*/
#include <roki2met.h>
#include <roki2global.h>

//Движения обеими ногами выполняются одновременно, поэтому делаем маски для левой и правой сторон
#define MASK_FOOT_FRONT (MASK_LEFT_FOOT_FRONT | MASK_RIGHT_FOOT_FRONT)
#define MASK_FOOT_SIDE  (MASK_LEFT_FOOT_SIDE | MASK_RIGHT_FOOT_SIDE)

//Определяет, за сколько фреймов исполнять программу на каждом шаге
int frameCount;

// factor must be from 1 to 10 integer

void turn(int direction, int factor) {
    frameCount = 4;
    if (direction == 1) {
        sfPoseGroup(MASK_RIGHT_PELVIC, -200 * factor, frameCount);
        sfPoseGroup(MASK_LEFT_PELVIC, 200 * factor, frameCount);
    }
    if (direction == -1) {
        sfPoseGroup(MASK_RIGHT_PELVIC, 200 * factor, frameCount);
        sfPoseGroup(MASK_LEFT_PELVIC, -200 * factor, frameCount);
    }
    sfPoseGroup(MASK_FOOT_SIDE, 200 * factor, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_RIGHT_PELVIC, 0, frameCount);
    sfPoseGroup(MASK_LEFT_PELVIC, 0, frameCount);
    sfPoseGroup(MASK_RIGHT_FOOT_SIDE, 0, frameCount);
    sfPoseGroup(MASK_LEFT_FOOT_SIDE, 0, frameCount);
    sfWaitFrame(frameCount);
    sfWaitFrame(6);
}

void jump_forward(int factor) {
    int current_foot_front = (rightFootFrontTarget + leftFootFrontTarget) / 2;
    frameCount = 30;
    sfPoseGroup(MASK_FOOT_FRONT, 16 * factor + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
    frameCount = 9;
    sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, -48 * factor + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 16 * factor + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 0 + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
}

void jump_backward(int factor) {
    int current_foot_front = (rightFootFrontTarget + leftFootFrontTarget) / 2;
    frameCount = 30;
    sfPoseGroup(MASK_FOOT_FRONT, -16 * factor + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
    frameCount = 9;
    sfPoseGroup(MASK_FOOT_SIDE, 1000, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 48 * factor + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, -16 * factor + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_FOOT_FRONT, 0 + current_foot_front, frameCount);
    sfWaitFrame(frameCount);
}

void jump_left(int factor) {
    frameCount = 30;
    sfPoseGroup(MASK_LEFT_FOOT_SIDE, -40 * factor, frameCount);
    sfPoseGroup(MASK_RIGHT_FOOT_SIDE, 40 * factor, frameCount);
    sfPoseGroup(MASK_LEFT_HIP_SIDE, -40 * factor, frameCount);
    sfPoseGroup(MASK_RIGHT_HIP_SIDE, 40 * factor, frameCount);
    sfWaitFrame(frameCount);
    frameCount = 9;
    sfPoseGroup(MASK_FOOT_SIDE, 200 * factor, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfPoseGroup(MASK_LEFT_HIP_SIDE, 0, frameCount);
    sfPoseGroup(MASK_RIGHT_HIP_SIDE, 0, frameCount);
    sfWaitFrame(frameCount);
    sfWaitFrame(frameCount);
}

void jump_right(int factor) {
    frameCount = 30;
    sfPoseGroup(MASK_LEFT_FOOT_SIDE, 40 * factor, frameCount);
    sfPoseGroup(MASK_RIGHT_FOOT_SIDE, -40 * factor, frameCount);
    sfPoseGroup(MASK_LEFT_HIP_SIDE, 40 * factor, frameCount);
    sfPoseGroup(MASK_RIGHT_HIP_SIDE, -40 * factor, frameCount);
    sfWaitFrame(frameCount);
    frameCount = 9;
    sfPoseGroup(MASK_FOOT_SIDE, 200 * factor, frameCount);
    sfWaitFrame(frameCount);
    sfPoseGroup(MASK_FOOT_SIDE, 0, frameCount);
    sfPoseGroup(MASK_LEFT_HIP_SIDE, 0, frameCount);
    sfPoseGroup(MASK_RIGHT_HIP_SIDE, 0, frameCount);
    sfWaitFrame(frameCount);
    sfWaitFrame(frameCount);
}

void main() {
  if (jump_mode % 10 == 1) jump_forward(jump_mode / 10);
  if (jump_mode % 10 == 2) jump_backward(jump_mode / 10);
  if (jump_mode % 10 == 3) jump_right(jump_mode / 10);
  if (jump_mode % 10 == 4) jump_left(jump_mode / 10);
  if (jump_mode % 10 == 5) turn(1, jump_mode / 10);
  if (jump_mode % 10 == 6) turn(-1, jump_mode / 10);
  jump_mode = 0;
  //Вот этим мы обеспечиваем переход обратно
  sfStartSlot( sfSlotIndex("roki2Walk_Remote.cpp") );
  }
