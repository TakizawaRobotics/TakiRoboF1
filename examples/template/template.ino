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
  Serial.print("degree");
  Serial.println(trf.getAzim());
  delay(200);
}
