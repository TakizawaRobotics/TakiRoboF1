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
  robot.lineUpdate();
  Serial.print("front:");
  Serial.println(robot.getLine(1));
  Serial.print("right:");
  Serial.println(robot.getLine(2));
  Serial.print("back:");
  Serial.println(robot.getLine(3));
  Serial.print("left:")
  Serial.println(robot.getLine(4));
}
