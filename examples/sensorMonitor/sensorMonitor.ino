#include <takiroboF1.h>

//HMC5883L(地磁気), MPU(ジャイロ), NONE(無し) から選択する
//何も指定しない場合, コンパスは使用されない
takiroboF1 robot(MPU);

void setup()
{
  Serial.begin(9600);
  robot.init();
}

void loop()
{
  char str[250];//文字列を入れる配列の生成
  robot.sensorMonitor(str);//アドレスを渡してセンサの文字列を取得する
  Serial.println(str);//文字列を表示する
  delay(500);
}