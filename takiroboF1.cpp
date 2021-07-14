#include "takiroboF1.h"
#include <avr/io.h>
#include <Wire.h>

#define MT1CW           3
#define MT1CCW          5
#define MT2CW           10
#define MT2CCW          9
#define MT3CW           11
#define MT3CCW          6
#define CW              1  //not pin number
#define CCW             2 //not pin number
#define ECHO            12
#define TRIG            13
#define LINE_PIN        1
#define IR1             8
#define IR2             17
#define IR3             16
#define IR4             14
#define LED             4
#define FRAMENUM        5
#define HMC5883L_ADDR   0x0D
#define LINE_ADDR       0x08

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
  static int mt_state[3];
  double mt_power[6];

  double spd[3] = {spd1, spd2, spd3};
  for (int i = 0; i < 3; i++)
  {
    if (spd[i] > 0)
    {
      mt_power[i * 2] = spd[i];
      mt_power[i * 2 + 1] = 0;
      if (mt_state[i] == CCW)
      {
        mt_state[i] = CW;
      }
      mt_state[i] = 1;
    }
    else if (spd[i] < 0)
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = -spd[i];
      if (mt_state[i] == CW)
      {
        mt_state[i] = CCW;
      }
    }
    else
    {
      mt_power[i * 2] = 0;
      mt_power[i * 2 + 1] = 0;
      mt_state[i] = 0;
    }
  }
  
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011011;
  TCCR2A = 0b10100011;
  TCCR2B = 0b00000100;
  ICR1 = 255;

  OCR2B = (unsigned int)(mt_power[0]); //3  MT1CW  980hz
  OCR0B = (unsigned int)(mt_power[1]); //5  MT1CCW 980hz
  OCR1A = (unsigned int)(mt_power[3]); //(1020*(mt_power[3]/255));//10  MT2CW   490hz
  OCR1B = (unsigned int)(mt_power[2]); //(1020*(mt_power[2]/255));//9   MT2CCW  490hz
  OCR2A = (unsigned int)(mt_power[4]); //11 MT3CW  980hz
  OCR0A = (unsigned int)(mt_power[5]); //6  MT3CCW 980hz
}

double robot::getUSS()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long durationMS = pulseIn(ECHO, HIGH);
  double getUSSance = (durationMS / 2.0) * 0.034;
  if ((getUSSance == 2) || (getUSSance > 400))
  {
    return -1.0;
  }
  else
  {
    return getUSSance;
  }
}

int robot::getLine(int num)
{
  return line[num - 1];
}

void robot::lineUpdate()
{
  Wire.requestFrom(LINE_ADDR, 4);
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

float robot::getStartAzim()
{
  return flontDeg;
}

float robot::getAzim()
{
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);
  while (Wire.available())
  {
    rawData[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    rawData[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    rawData[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
  }
  int data[2] = {};
  data[0] = (rawData[0] - medianX);
  data[1] = (rawData[1] - medianY) * scale;
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
  digitalWrite(MT1CW, LOW);
  digitalWrite(MT1CCW, LOW);
  digitalWrite(MT2CW, LOW);
  digitalWrite(MT2CCW, LOW);
  digitalWrite(MT3CW, LOW);
  digitalWrite(MT3CCW, LOW); //つまりmotor(0, 0, 0)と同じ
  digitalWrite(LED, HIGH); //割り込み中はLED2が点灯します。
  initComp = true;
}

void robot::init()
{
  attachInterrupt(0, interrupt, LOW);
  pinMode(LED, OUTPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(MT1CW, OUTPUT);
  pinMode(MT1CCW, OUTPUT);
  pinMode(MT2CW, OUTPUT);
  pinMode(MT2CCW, OUTPUT);
  pinMode(MT3CW, OUTPUT);
  pinMode(MT3CCW, OUTPUT);
  digitalWrite(MT1CW, LOW);
  digitalWrite(MT1CCW, LOW);
  digitalWrite(MT2CW, LOW);
  digitalWrite(MT2CCW, LOW);
  digitalWrite(MT3CW, LOW);
  digitalWrite(MT3CCW, LOW);
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);
  Wire.endTransmission();
  boolean calibComp = false;
  getAzim();
  flontDeg = getAzim();
}