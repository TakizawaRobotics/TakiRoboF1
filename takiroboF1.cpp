#include "takiroboF1.h"
#include <avr/io.h>
#include <Wire.h>

#define MT1CW           3
#define MT1CCW          5
#define MT2CW           10
#define MT2CCW          9
#define MT3CW           11
#define MT3CCW          6
#define CW              1 //not pin number
#define CCW             2 //not pin number
#define ECHO            12
#define TRIG            13
#define LINE_PIN        1
#define IR1             8
#define IR2             17
#define IR3             16
#define IR4             14
#define A1              1  //同じピン↓
#define D1              15 //同じピン↑
#define D2              7  //digitalPin
#define D3              4  //digitalPin
#define LED             4
#define FRAMENUM        5
#define HMC5883L_ADDR   0x0D
#define LINE_ADDR       0x08

int *pindata;
volatile bool startLoop;

takiroboF1::takiroboF1(float median_x, float median_y, float scale)
{
  MEDIAN_x = median_x;
  MEDIAN_y = median_y;
  SCALE = scale;
  pindata = _pinData;
  startLoop = false;
}

takiroboF1::takiroboF1(float median_x, float median_y, float scale,int* _pinstate)
{
  MEDIAN_x = median_x;
  MEDIAN_y = median_y;
  SCALE = scale;
  for (int i = 0; i < 3;i++)
  {
    _pinData[i] = _pinstate[i];
  }
    pindata = _pinData;
  startLoop = false;
}

takiroboF1::takiroboF1(int* _pinstate)
{
  MEDIAN_x = 0;
  MEDIAN_y = 0;
  SCALE = 1;
  for (int i = 0; i < 3;i++)
  {
    _pinData[i] = _pinstate[i];
  }
    pindata = _pinData;
  if(pindata[2]!=0)
  {
    startLoop = false;
  }else{
    startLoop = true;
  }
}

takiroboF1::takiroboF1()
{
  MEDIAN_x = 0;
  MEDIAN_y = 0;
  SCALE = 1;
  pindata = _pinData;
  startLoop = true;
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
  return line[num - 1];
}

void takiroboF1::lineUpdate()
{
  Wire.requestFrom(LINE_ADDR, 4);
  int i = 0;
  while (Wire.available())
  {
    line[i] = Wire.read();
    i++;
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

float takiroboF1::getStartingAzimuth()
{
  return starting_position_deg;
}

float takiroboF1::getAzimuth()
{
  delay(200);
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);
  while (Wire.available())
  {
    raw_data[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    raw_data[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    raw_data[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
  }
  int data[2] = {};
  data[0] = (raw_data[0] - MEDIAN_x);
  data[1] = (raw_data[1] - MEDIAN_y) * SCALE;
  float deg=atan2(data[1], data[0]) * 180.0 / PI;
  if (deg<0)
  {
    deg=deg + 360;
  }
  return deg;
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
  Serial.println("キャリブレーションが終了しました。");
  Serial.print("(");
  Serial.print(MEDIAN_x);
  Serial.print(",");
  Serial.print(MEDIAN_y);
  Serial.print(",");
  Serial.print(SCALE);
  Serial.println(")");
}

int takiroboF1::getAnalogPin()
{
  if(pindata[0]==3)
  {
    return analogRead(A1);
  }else{
    return 0;
  }
}

int takiroboF1::getDigitalPin(int pin)
{
  switch(pin)
  {
    case 1:
      if(pindata[0]==1)
      {
        pinMode(D1, INPUT);
        return digitalRead(D1);      
      }else{
        return 0;
      }
      break;
    case 2:
      if(pindata[1]==1)
      {
        pinMode(D2, INPUT);
        return digitalRead(D2);        
      }else{
        return 0;
      }
      break;
    case 3:
      if (pindata[2]==1)
      {
        pinMode(D3, INPUT);
        return digitalRead(D3);        
      }else{
        return 0;
      }
      break;
    default:
      return 0;
      break;
  }
}

void takiroboF1::setDigitalPin(int pin,bool value)
{
  switch(pin)
  {
    case 1:
      if(pindata[0]==2)
      {
        pinMode(D1,OUTPUT);
        digitalWrite(D1,value);        
      }
      break;
    case 2:
      if(pindata[1]==2)
      {
        pinMode(D2,OUTPUT);
        digitalWrite(D2,value);        
      }
      break;
    case 3:
      if(pindata[2]==2)
      {
        pinMode(D3,OUTPUT);
        digitalWrite(D3,value);        
      }
      break;
    default:
      break;
  }
}

static void takiroboF1::interrupt()
{
  pinMode(2, INPUT);
  startLoop = 0;
  digitalWrite(MT1CW, LOW);
  digitalWrite(MT1CCW, LOW);
  digitalWrite(MT2CW, LOW);
  digitalWrite(MT2CCW, LOW);
  digitalWrite(MT3CW, LOW);
  digitalWrite(MT3CCW, LOW); //つまりmotor(0, 0, 0)と同じ
  while(1)
  {

    if(pindata[2]==0)
    {
      digitalWrite(LED, LOW); //割り込み中はLED2が消灯します。    
    }
    if(digitalRead(2)==HIGH)
    {
      break;
    }
  }
  if(pindata[2]==0)
  {
    digitalWrite(LED,HIGH);
  }
}

void takiroboF1::init()
{

  Serial.begin(9600);

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

  attachInterrupt(0, interrupt, LOW);

  digitalWrite(MT1CW, LOW);
  digitalWrite(MT1CCW, LOW);
  digitalWrite(MT2CW, LOW);
  digitalWrite(MT2CCW, LOW);
  digitalWrite(MT3CW, LOW);
  digitalWrite(MT3CCW, LOW);

  Wire.begin();
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);
  Wire.endTransmission();

  if (startLoop)
  {
    calib_compass();
    for (int i = 0; i <= 10; i++)
    {
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(100);      
    }
    digitalWrite(LED, HIGH);
  }

  while(startLoop)
  {
  }

  attachInterrupt(0, interrupt, FALLING);

  starting_position_deg = getAzimuth();

}
