/*
   モーターを動かして、機体を時計回りに回転させるだけのプログラムです。
*/
#include "takiroboF1.h"
takiroboF1 robot;
void setup() {
  // put your setup code here, to run once:
  robot.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 256; i++) {
    robot.motor(i, i, i);
    delay(40);
  }
  for (int i = 255; i >= 0; i--) {
    robot.motor(i, i, i);
    delay(40);
  }
}
