#include "takiroboF1.h"
#include <avr/io.h>
#include <Wire.h>

#define mt1cw 3
#define mt1ccw 5
#define mt2cw 10
#define mt2ccw 9
#define mt3cw 11
#define mt3ccw 6
#define cw 1  //not pin number
#define ccw 2 //not pin number
#define echo 12
#define trig 13
#define linePin 1
#define IR1 8
#define IR2 17
#define IR3 16
#define IR4 14
#define LED 4
#define framenum 5
#define addr 0x0D

boolean initComp = false;

robot::takiroboF1(float MedianX, float MedianY, float Scale)
{
  medianX = medianY;
  medianY = MedianY;
  scale = scale;
}

robot::takiroboF1()
{
  medianX = 0;
  medianY = 0;
  scale = 1;
}

void robot::motor(double spd1, double spd2, double spd3)
{
  //int offtime = 0;
  static int mt_state[3];
  double mt_power[6];

  double spd[3] = {spd1, spd2, spd3};
  for (int i = 0; i < 3; i++)
  {
    if (spd[i] > 0)
    {
      mt_power[i * 2] = spd[i];
      mt_power[i * 2 + 1] = 0;
      if (mt_state[i] == ccw)
      {
        //offtime = 1;
        mt_state[i] = cw;
      }
      mt_state[i] = 1;
    }
    else if (spd[i] < 0)
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = -spd[i];
      if (mt_state[i] == cw)
      {
        //offtime = 1;
        mt_state[i] = ccw;
      }
    }
    else
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = 0;
      mt_state[i] = 0;
    }
  }
  /*
  if (offtime == 1)
  {
    digitalWrite(mt1cw, LOW);
    digitalWrite(mt1ccw, LOW);
    digitalWrite(mt2cw, LOW);
    digitalWrite(mt2ccw, LOW);
    digitalWrite(mt3cw, LOW);
    digitalWrite(mt3ccw, LOW);
    delayMicroseconds(100);
  }
  */
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011011;
  TCCR2A = 0b10100011;
  TCCR2B = 0b00000100;
  ICR1 = 255;

  OCR2B = (unsigned int)(mt_power[0]); //3  mt1cw  980hz
  OCR0B = (unsigned int)(mt_power[1]); //5  mt1ccw 980hz
  OCR1A = (unsigned int)(mt_power[3]); //(1020*(mt_power[3]/255));//10  mt2cw   490hz
  OCR1B = (unsigned int)(mt_power[2]); //(1020*(mt_power[2]/255));//9   mt2ccw  490hz
  OCR2A = (unsigned int)(mt_power[4]); //11 mt3cw  980hz
  OCR0A = (unsigned int)(mt_power[5]); //6  mt3ccw 980hz
}

double robot::dist()
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long durationMS = pulseIn(echo, HIGH);
  double distance = durationMS / 2.0 * 0.034;
  if (distance == 2 || distance > 400)
  {
    return -1.0;
  }
  else
  {
    return distance;
  }
}

int robot::getLine(int num)
{
  return line[num - 1];
}

void robot::lineUpdate()
{
  Wire.requestFrom(8, 4);
  int i = 0;
  while (Wire.available())
  {
    line[i] = Wire.read();
    i++;
  }
}

int robot::getIr(int num)
{
  switch (num)
  {
  case 1:
    return _ir1;
    break;
  case 2:
    return _ir2;
    break;
  case 3:
    return _ir3;
    break;
  case 4:
    return _ir4;
    break;
  default:
    return 0;
    break;
  }
}

void robot::irUpdate()
{
  for (int i = 0; i < 500; i++)
  {
    if (digitalRead(IR1) == LOW)
    {
      _ir1++;
    }
  }
  for (int i = 0; i < 500; i++)
  {
    if (digitalRead(IR2) == LOW)
    {
      _ir2++;
    }
  }
  for (int i = 0; i < 500; i++)
  {
    if (digitalRead(IR3) == LOW)
    {
      _ir3++;
    }
  }

  for (int i = 0; i < 500; i++)
  {
    if (digitalRead(IR4) == LOW)
    {
      _ir4++;
    }
  }
}

float robot::getFlontAzim()
{
  return flontDeg;
}

float robot::getAzim()
{
  Wire.beginTransmission(addr);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(addr, 6);
  while (Wire.available())
  {
    rawData[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    rawData[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    rawData[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
  }
  int data[2] = {};
  data[0] = (rawData[0] - medianX);
  data[1] = (rawData[1] - medianY) * scale;
  //Serial.println(atan2(data[1], data[0]) * 180.0 / PI);
  return atan2(data[1], data[0]) * 180.0 / PI;
}

void robot::calibCompass()
{
  int start = millis();
  int finish;
  int nowCalib = 0;
  float median[2] = {};
  float calibData[4] = {};
  float scale = 0;
  while (1)
  {
    dataGet();
    delay(200);
    Serial.println("robot is calibrating now");
    if (rawData[0] < calibData[0])
    {
      calibData[0] = rawData[0];
      nowCalib = 1;
      //Xmin
    }
    if (rawData[0] > calibData[1])
    {
      calibData[1] = rawData[0];
      nowCalib = 1;
      //Xmax
    }
    if (rawData[1] < calibData[2])
    {
      calibData[2] = rawData[1];
      nowCalib = 1;
      //Ymin
    }
    if (rawData[1] > calibData[3])
    {
      calibData[3] = rawData[1];
      nowCalib = 1;
      //Ymax
    }
    finish = millis();
    if ((finish - start) > 8000 && nowCalib == 0)
    {
      break;
    }
    nowCalib = 0;
  }
  median[0] = (calibData[0] + calibData[1]) / 2;
  median[1] = (calibData[2] + calibData[3]) / 2;
  scale = ((calibData[1] - calibData[0]) / (calibData[3] - calibData[2]));
  Serial.print("(");
  Serial.print(median[0]);
  Serial.print(",");
  Serial.print(median[1]);
  Serial.print(",");
  Serial.print(scale);
  Serial.print(")");
}

void interrupt()
{
  digitalWrite(mt1cw, LOW);
  digitalWrite(mt1ccw, LOW);
  digitalWrite(mt2cw, LOW);
  digitalWrite(mt2ccw, LOW);
  digitalWrite(mt3cw, LOW);
  digitalWrite(mt3ccw, LOW); //つまりmotor(0, 0, 0)と同じ
  //digitalWrite(LED, LOW);    //割り込み中はLED2が消えます。
  digitalWrite(LED, HIGH); //割り込み中はLED2が点灯します。
  initComp = true;
}

void robot::initialize()
{
  attachInterrupt(0, interrupt, LOW);
  pinMode(LED, OUTPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(mt1cw, OUTPUT);
  pinMode(mt1ccw, OUTPUT);
  pinMode(mt2cw, OUTPUT);
  pinMode(mt2ccw, OUTPUT);
  pinMode(mt3cw, OUTPUT);
  pinMode(mt3ccw, OUTPUT);
  digitalWrite(mt1cw, LOW);
  digitalWrite(mt1ccw, LOW);
  digitalWrite(mt2cw, LOW);
  digitalWrite(mt2ccw, LOW);
  digitalWrite(mt3cw, LOW);
  digitalWrite(mt3ccw, LOW);
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(addr);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(addr);
  Wire.write(0x09);
  Wire.write(0x1D);
  Wire.endTransmission();
  boolean calibComp = false;
  getAzim();
  flontDeg = getAzim();
}