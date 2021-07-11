#include <takiroboF1.h>

takiroboF1 trf(x,y,s); //()の中は任意で書き変えてください。詳細は取扱説明書参照。

void setup()
{
  trf.initialize();
  //ここは電源を入れてから一度しか実行されません。
  Serial.begin(9600);
}

void loop()
{
  //プログラムはここをループし続けます。
  trf.lineUpdate();
  Serial.print("flont:");
  Serial.println(trf.getLine(1));
  Serial.print("right:");
  Serial.println(trf.getLine(2));
  Serial.print("back:");
  Serial.println(trf.getLine(3));
  Serial.print("left:")
  Serial.println(trf.getLine(4));
}
