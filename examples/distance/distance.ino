#include <takiroboF1.h>

//HMC5883L(地磁気), MPU6050(ジャイロ), NONE(無し) から選択する
//何も指定しない場合, コンパスは使用されない
takiroboF1 robot;

void setup()
{
  Serial.begin(9600);
  robot.init();
  //ここは電源を入れてから一度しか実行されません。
}

void loop()
{
  //プログラムはここをループし続けます。
  if(robot.getBtn() == HIGH)//スタートスイッチが押された時
  {
    Serial.print("距離:");
    Serial.println(robot.getUSS());
  }
}
