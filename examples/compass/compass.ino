#include <takiroboF1.h>

//HMC5883L(地磁気), MPU(ジャイロ), NONE(無し) から選択する
//何も指定しない場合, コンパスは使用されない
takiroboF1 robot(MPU);

void setup()
{
  //ここは電源を入れてから一度しか実行されません。
  Serial.begin(9600);
  robot.init();
}

void loop()
{
  //プログラムはここをループし続けます。
  if(robot.getBtn() == HIGH)//スタートスイッチが押された時
  {
    Serial.print("方位角：");
    Serial.println(robot.getAzimuth());
  }
}
