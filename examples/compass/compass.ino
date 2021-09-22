#include <takiroboF1.h>

takiroboF1 robot(x,y,s); //()の中は任意で書き変えてください。詳細は取扱説明書参照。

void setup()
{
  robot.init();
  //ここは電源を入れてから一度しか実行されません。
}

void loop()
{
  //プログラムはここをループし続けます。
  Serial.print("degree");
  Serial.println(robot.getAzimuth());
  delay(200);
}
