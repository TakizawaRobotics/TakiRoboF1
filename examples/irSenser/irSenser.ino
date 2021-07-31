#include <takiroboF1.h>

takiroboF1 robot;

void setup()
{
  robot.init();
  //ここは電源を入れてから一度しか実行されません。
  Serial.begin(9600);
}

void loop()
{
  //プログラムはここをループし続けます。
  robot.irUpdate();
  Serial.print("front:");
  Serial.println(robot.getIr(1));
  Serial.print("right:");
  Serial.println(robot.getIr(2));
  Serial.print("back:");
  Serial.println(robot.getIr(3));
  Serial.print("left:")；
  Serial.println(robot.getIr(4));
}
