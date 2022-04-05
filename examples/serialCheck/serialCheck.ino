#include <takiroboF1.h>

takiroboF1 robot; //()の中は任意で書き変えてください。詳細は取扱説明書またはREADME.mdを参照。

void setup()
{
  robot.init(); //この行は消さないでください。
  //ここは電源を入れてから一度しか実行されません。
  Serial.println("プログラムは問題なく書き込めています。");

}

void loop()
{
  //プログラムはここをループし続けます。
  Serial.println("Hello World!!");

}
