/*
   コンパスセンサのキャリブレーションに使う値を取得するプログラムです。
*/
#include "takiroboF1.h"

takiroboF1 trf;

void setup() {
  // put your setup code here, to run once:
  trf.init();
  trf.calib_compass();
}

void loop() {
  // put your main code here, to run repeatedly:

}
