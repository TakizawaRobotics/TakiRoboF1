#include <takiroboF1.h>

//HMC5883L(地磁気), MPU6050(ジャイロ), NONE(無し) から選択する
//何も指定しない場合, コンパスは使用されない
takiroboF1 robot(HMC5883L);

void setup()
{
  //ここは電源を入れてから一度しか実行されません。
  Serial.begin(9600);
  robot.init();
}

void loop()
{
  //EEPROMにキャリブレーションデータを書き込み。
  robot.calibCompass();
  Serial.println("fin");
}