#include <takiroboF1.h>

takiroboF1 robot; //()の中は任意で書き変えてください。詳細は取扱説明書またはREADME.mdを参照。

void setup()
{
  robot.init();
  //ここは電源を入れてから一度しか実行されません。
}

void loop()
{
  //プログラムはここをループし続けます。
  int deg = 90; //ロボットに進ませたい角度を代入する。
  int spd = 255; //スピードを入れる。
  robot.mtDeg(deg, spd);
  delay(2000); //2秒間、deg方向に進み続ける。
  robot.motor(0, 0, 0);
}