/*
  コンパスセンサのキャリブレーションに使う値を取得するプログラムです。
  このプログラムを実行する場合、プログラム開始用スイッチはOFFの状態にしてください。
*/
#include "takiroboF1.h"

takiroboF1 robot;

void setup() {
  // put your setup code here, to run once:
  robot.init();
  robot.calib_compass();
}

void loop() {
  // put your main code here, to run repeatedly:

}
