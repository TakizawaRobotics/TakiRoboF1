/*======================================================================
Project Name    : takiroboF1
File Name       : takiroboF1.cpp
Encoding        : utf-8
Creation Date   : c++
author          : Takumi Yoneda, Hayato Tajiri
update date     : 2021/11/13 
 
Copyright © <2021> TakizawaRoboticsLLC. All rights reserved.
 
This source code or any portion thereof must not be  
reproduced or used in any manner whatsoever.
======================================================================*/

#include "takiroboF1.h"
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <Wire.h>

#define MT1CW           3
#define MT1CCW          5
#define MT2CW           10
#define MT2CCW          9
#define MT3CW           11
#define MT3CCW          6
#define CW              1 
#define CCW             2 
#define ECHO            12
#define TRIG            13
#define IR1             8
#define IR2             17
#define IR3             16
#define IR4             14
#define LED             4
#define RXPIN           15
#define TXPIN           7
#define HMC5883L_ADDR   0x0D

SoftwareSerial mySerial(RXPIN, TXPIN); // RX TX

volatile int readyToStart;

takiroboF1::takiroboF1()
{
    MEDIAN_x = 0;
    MEDIAN_y = 0;
    SCALE = 1;
    calib = 0;
    firstAzim = 0;
    readyToStart = 0;
}

void takiroboF1::motor(double spd1, double spd2, double spd3)
{
  int mt_state[3];
  double mt_power[6];
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011011;
  TCCR2A = 0b10100011;
  TCCR2B = 0b00000100;
  ICR1 = 255;
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

  OCR2B = (unsigned int)(mt_power[0]); //3  MT1CW  980hz
  OCR0B = (unsigned int)(mt_power[1]); //5  MT1CCW 980hz
  OCR1A = (unsigned int)(mt_power[3]); //(1020*(mt_power[3]/255));//10  MT2CW   490hz
  OCR1B = (unsigned int)(mt_power[2]); //(1020*(mt_power[2]/255));//9   MT2CCW  490hz
  OCR2A = (unsigned int)(mt_power[4]); //11 MT3CW  980hz
  OCR0A = (unsigned int)(mt_power[5]); //6  MT3CCW 980hz
}

void takiroboF1::mtDeg(int deg, int spd)
{
  double mt[3];
  double maxValue = 0;
  mt[0] = sin(((deg - 60) / 180) * M_PI) * spd;
  mt[1] = sin(((deg - 180) / 180) * M_PI) * spd;
  mt[2] = sin(((deg - 300) / 180) * M_PI) * spd;
  maxValue = max(max(mt[0], mt[1]), mt[2]);
  for (int i = 0; i < 3; i++)
  {
    mt[i] = mt[i] / maxValue * spd;
  }
  motor(-mt[0], -mt[1], -mt[2]);
}

double takiroboF1::getUSS()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long duration_ms = pulseIn(ECHO, HIGH);
  double distance = (duration_ms / 2.0) * 0.034;
  if ((distance == 2) || (distance > 400))
  {
    return -1.0;
  }
  else
  {
    return distance;
  }
}

int takiroboF1::getLine(int num)
{
  if (mySerial.available() > 100)
  {
    mySerial.flush();
  }
  switch (num)
  {
  case 1:
    // If this program execute, you want to get line sensor of front.
    mySerial.write("f");
    while (mySerial.available() == 0)
      ;
    while (mySerial.available() > 0)
    {
      return mySerial.read();
    }
    break;
  case 2:
    // If this program execute, you want to get line sensor of right.
    mySerial.write("r");
    while (mySerial.available() == 0)
      ;
    while (mySerial.available() > 0)
    {
      return mySerial.read();
    }
    break;
  case 3:
    // If this program execute, you want to get line sensor of back.
    mySerial.write("b");
    while (mySerial.available() == 0)
      ;
    while (mySerial.available() > 0)
    {
      return mySerial.read();
    }
    break;
  case 4:
    // If this program execute, you want to get line sensor of left.
    mySerial.write("l");
    while (mySerial.available() == 0)
      ;
    while (mySerial.available() > 0)
    {
      return mySerial.read();
    }
    break;
  default:
    return 0;
    break;
  }
}

int takiroboF1::getIr(int num)
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

void takiroboF1::irUpdate()
{
  _ir1 = 0;
  _ir2 = 0;
  _ir3 = 0;
  _ir4 = 0;

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

void takiroboF1::azimUpdate()
{

    /*初期化*/
    raw_data[0] = 0;
    raw_data[1] = 0;
    raw_data[2] = 0;
    int data[2] = {};
    float deg = 0;
    static float pre_deg = 0;
    static long pre_millis = 0;

    /*前回の読み出しから200ms秒以上経っていたら*/
    if(millis() - pre_millis > 200)//前回の読み出しから200ms待つ
    {
      /*アドレス送信*/
      Wire.beginTransmission(HMC5883L_ADDR);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(HMC5883L_ADDR, 6);

      /*I2Cから数値を取得する*/
      while (Wire.available())
      {
        raw_data[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
        raw_data[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
        raw_data[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
      }

      /*正しく取得できていなかったら*/
      if((raw_data[0] == 0) || (raw_data[1] == 0) || (raw_data[2]  == 0))//エラー処理
      {
        deg = pre_deg;//過去に取得した角度を返す
      }
      /*正常に取得できた場合*/
      else
      {
        /*オイラー角度に変換する*/
        data[0] = (raw_data[0] - MEDIAN_x);
        data[1] = (raw_data[1] - MEDIAN_y) * SCALE;
        deg=atan2(data[1], data[0]) * 180.0 / PI;
        if (deg<0)
        {
          deg += 360;
        }
      }

      /*現在の時間を保存する*/
      pre_millis = millis();
    }
    /*前回の読み出しから200ms秒以上経っていなかったら*/
    else
    {
      deg = pre_deg;//過去の数値を返す
    }

    /*現在のデータを保存する*/
    pre_deg = deg;
    degree = deg;
    return degree;
}

double takiroboF1::getAzimuth()
{
    azimUpdate();
    double deg = 0;
    deg = degree - firstAzim;
    if(deg<0)
    {
        deg = deg + 360;
    }
}

void takiroboF1::calib_compass()
{
    //Serial.println("Robot start calibrating 5 seconds later");
    Serial.println("キャリブレーションは5秒後に開始されます。");
    int start = millis();
    int finish;
    int now_calib = 0;
    float median[2] = {};
    float calib_data[4] = {};
    float SCALE = 0;
    while(1)
    {
      finish = millis();
      if((finish-start)>=5000)
      {
        break;
      }
    }
    start = millis();
    finish = 0;
    while (1)
    {
      getAzimuth();
      delay(10);
      //Serial.println("Robot is calibrating now");
      Serial.println("現在、キャリブレーション中です。");
      if (raw_data[0] < calib_data[0])
      {
        calib_data[0] = raw_data[0];
        now_calib = 1;
        //Xmin
      }
      if (raw_data[0] > calib_data[1])
      {
        calib_data[1] = raw_data[0];
        now_calib = 1;
        //Xmax
      }
      if (raw_data[1] < calib_data[2])
      {
        calib_data[2] = raw_data[1];
        now_calib = 1;
        //Ymin
      }
      if (raw_data[1] > calib_data[3])
      {
        calib_data[3] = raw_data[1];
        now_calib = 1;
        //Ymax
      }
      finish = millis();
      if ((finish - start) > 8000 && now_calib == 0)
      {
        break;
      }
      now_calib = 0;
    }
    MEDIAN_x = (calib_data[0] + calib_data[1]) / 2;
    MEDIAN_y = (calib_data[2] + calib_data[3]) / 2;
    SCALE = ((calib_data[1] - calib_data[0]) / (calib_data[3] - calib_data[2]));

    //Serial.println("calibration complete");
    /*
    Serial.println("キャリブレーションが終了しました。");
    Serial.print("(");
    Serial.print(MEDIAN_x);
    Serial.print(",");
    Serial.print(MEDIAN_y);
    Serial.print(",");
    Serial.print(SCALE);
    Serial.println(")");
    */
}

static void takiroboF1::interrupt()
{
    pinMode(2, INPUT);
    digitalWrite(MT1CW, LOW);
    digitalWrite(MT1CCW, LOW);
    digitalWrite(MT2CW, LOW);
    digitalWrite(MT2CCW, LOW);
    digitalWrite(MT3CW, LOW);
    digitalWrite(MT3CCW, LOW);
    while (1)
    {
        digitalWrite(LED, LOW);
        if (digitalRead(2) == HIGH)
        {
          break;
        }
    }
    attachInterrupt(0, interrupt,LOW);
    digitalWrite(LED, HIGH);
    readyToStart = 1;
}

void takiroboF1::init()
{

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

    Serial.begin(9600);
    mySerial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x09);
    Wire.write(0x1D);
    Wire.endTransmission();

    attachInterrupt(0, interrupt,LOW);

    while(readyToStart == 0)
    {
        if(calib == 0)
        {
            calib_compass();
            calib = 1;
        }
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
    }
    firstAzim = getAzimuth();
}