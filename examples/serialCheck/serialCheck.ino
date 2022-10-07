#include <takiroboF1.h>

//HMC5883L(地磁気), MPU(ジャイロ), NONE(無し) から選択する
//何も指定しない場合, コンパスは使用されない
takiroboF1 robot;
void setup()
{
  //ここは電源を入れてから一度しか実行されません。
  Serial.begin(9600);
  robot.init(); //この行は消さないでください。
  Serial.println("プログラムは問題なく書き込めています。");
}

void loop()
{
  //プログラムはここをループし続けます。
  if(robot.getBtn() == HIGH)//スタートスイッチが押された時
  {
    Serial.println("Hello World!!");
  }
}
