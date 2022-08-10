#include <takiroboF1.h>

//HMC5883L(地磁気), MPU6050(ジャイロ), NONE(無し) から選択する
//何も指定しない場合, コンパスは使用されない
takiroboF1 robot;

int deg = 90; //ロボットに進ませたい角度を代入する。
int spd = 255; //スピードを入れる。
int yaw = 30; //回転成分を入れる。
void setup()
{
  //ここは電源を入れてから一度しか実行されません。
  Serial.begin(9600);
  robot.init();
}

void loop()
{
  //プログラムはここをループし続けます。
  if(robot.getBtn() == HIGH){//スタートスイッチが押された時
    robot.omniControl(deg, spd, yaw);
  }
}
