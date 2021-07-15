#include <takiroboF1.h>

takiroboF1 robot(x,y,s); //()の中は任意で書き変えてください。詳細は取扱説明書参照。

void setup()
{
  robot.init();
  //ここは電源を入れてから一度しか実行されません。
  Serial.begin(9600);
}

void loop()
{
  //プログラムはここをループし続けます。
  Serial.print("getUSSance:");
  serial.println(robot.getUSS());
}
