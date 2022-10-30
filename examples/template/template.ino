#include <takiroboF1.h>

//HMC5883L(地磁気), MPU(ジャイロ), NONE(無し) から選択する
takiroboF1 robot;

void setup()
{
  Serial.begin(9600);
  robot.init();
  
}

void loop()
{
  if(robot.getBtn() == HIGH)
  {
    
  }
}
