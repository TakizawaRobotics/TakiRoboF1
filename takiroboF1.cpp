/*======================================================================
Project Name    : takiroboF1
File Name       : takiroboF1.cpp
Encoding        : utf-8
Creation Date   : c++
author          : Takumi Yoneda, Hayato Tajiri
update date     : 2022/10/7 
 
Copyright © <2022> TakizawaRoboticsLLC. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
======================================================================*/

#include "takiroboF1.h"
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <TimerOne.h>
#include <EEPROM.h>
#include "MPU6050_6Axis_MotionApps20.h"

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
#define BTN             2
#define HMC5883L_ADDR   0x0D
#define MPU6050_ADDR    0x68
#define OUTPUT_READABLE_YAWPITCHROLL


///////////////////////////////
/*グローバル変数*/
///////////////////////////////
volatile COMPASS Mode;
volatile static bool Ready_to_start;
volatile int Latest_azim;
volatile double Deg_mpu;
volatile double Degree;
volatile double Median_x;
volatile double Median_y;
volatile double Scale;
volatile int Raw_data[3];
volatile static int _ir1;
volatile static int _ir2;
volatile static int _ir3;
volatile static int _ir4;
volatile static int Uss_val;
volatile bool Calib;
volatile bool az_check_flag = false;

///////////////
/*コンストラクタ*/
////////////////
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


/////////////////////////////////////////////////
/*タイマー割り込み関数 2040マイクロ秒周期で呼び出される*/
/////////////////////////////////////////////////
void takiroboF1::timerISR(void){

  static long count = 0;

  //タイマ割り込みの停止
  Timer1.detachInterrupt();

  /*一定周期ごとに関数を実行*/
  /*タイミングが重なった場合上から優先で実行される*/
  if((count % 3) == 0){//約100Hz
    takiroboF1::azimUpdate();
  }
  else if((count % 23) == 0){//約20Hz
    takiroboF1::irUpdate();
  }
  else if((count % 59) == 0){//約10Hz
    takiroboF1::btnUpdate();
  }
  else if((count % 101) == 0){//約5Hz
    takiroboF1::ussUpdate();
  }
  else{
    /*DO NOTHING*/
  }

  count++;

  /*タイマー割り込みの再開*/
  Timer1.attachInterrupt(timerISR); 
}

/////////////////////////////
/*ラインセンサのUART接続の設定*/
////////////////////////////
SoftwareSerial lineSerial(RXPIN, TXPIN); // RX TX

////////////////
/*コンストラクタ*/
///////////////
takiroboF1::takiroboF1(void)
{
  Mode = NONE;
  Ready_to_start = true;
  Median_x = 0;
  Median_y = 0;
  Scale = 1;
  Calib = true;
  Latest_azim = 0;
}

takiroboF1::takiroboF1(COMPASS mode)
{
  Mode = mode;
  Ready_to_start = false;
  Median_x = 0;
  Median_y = 0;
  Scale = 1;
  Calib = false;
  Latest_azim = 0;
}

///////////////////////////////////////////
/*2輪モーター制御 　　　             　　　  */
/*メモ：+で前進する、ポート3は0にする -255~255*/
///////////////////////////////////////////
void takiroboF1::motor(int left, int right)
{
  int mt_state[3];
  int mt_power[6];
  int spd[3] = {left, right, 0};
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

/////////////////////////////////////////
/*3輪モーター制御 　　　             　　　*/
/*メモ：左手座標系で動作する(+方向で時計回り)*/
////////////////////////////////////////
void takiroboF1::motor(int spd1, int spd2, int spd3)
{
  int mt_state[3];
  int mt_power[6];
  int spd[3] = {spd1, spd2, spd3};
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

/////////////////////////////////////
/*3輪オムニホイール制御               */
/*入力値(進行方向の角度、スピード、回転)*/
/*メモ：左手座標系で動作する　　　　　　*/
///////////////////////////////////
void takiroboF1::omniControl(float deg, float spd, float yaw)
{
  int mt[3];
  int max_value = 0;
  int cut_off = 0;
  uint8_t i = 0;
  mt[0] = sin(((float)(deg - 60) / 180.0) * M_PI) * spd + yaw;
  mt[1] = sin(((float)(deg - 180) / 180.0) * M_PI) * spd + yaw;
  mt[2] = sin(((float)(deg - 300) / 180.0) * M_PI) * spd + yaw;
  max_value = max(max(mt[0], mt[1]), mt[2]);

  /*一番大きい数値の部分がspdの入力値になるように最大値変換する*/
  for (i = 0; i < 3; i++)
  {
    mt[i] =  (float) mt[i] / (float)max_value * spd ;
  }

  /*モーターを出力する*/
  motor(mt[0], mt[1], mt[2]);
  
}

///////////////////////////
/*超音波センサの距離を計算する*/
///////////////////////////
void takiroboF1::ussUpdate(void)
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long duration_ms = pulseIn(ECHO, HIGH);
  int distance = ((float)duration_ms / 2.0) * 0.034;
  if ((distance == 2) || (distance > 400))
  {
    Uss_val = -1.0;
  }
  else
  {
    Uss_val = distance;
  }
}

///////////////////////////
/*超音波センサの距離を取得する*/
///////////////////////////
int takiroboF1::getUSS(void)
{
  return Uss_val;
}

//////////////////////////////////
/*ラインセンサの数値を取得する　　　　*/
/*メモ：シリアル通信でデータを取得する*/
//////////////////////////////////
int takiroboF1::getLine(DIRECTION dir)
{
  int ret = 0;
  long last_time = 0;

  /*値の取得を失敗した場合は再度、取得を行う*/
  while(1){
    /*シリアルバッファの削除*/
    lineSerial.flush();

    switch (dir)
    {
    case FRONT:
      // If this program execute, you want to get line sensor of front.
      lineSerial.write("f");
      break;
    case RIGHT:
      // If this program execute, you want to get line sensor of right.
      lineSerial.write("r");
      break;
    case BACK:
      // If this program execute, you want to get line sensor of back.
      lineSerial.write("b");
      break;
    case LEFT:
      // If this program execute, you want to get line sensor of left.
      lineSerial.write("l");
      break;
    default:
      ret = 0;
      break;
    }

    /*エラー処理(主に通信ができなかった場合)*/
    
    last_time = millis();
    while (lineSerial.available() == 0){
      if((millis() - last_time) > 100)
      {
        ret = -1;
        break;
      }
    }

    /*ラインセンサ取得*/
    while (lineSerial.available() > 0)
    {
      ret = (int)lineSerial.read();
    }

    /*値の取得に成功したら終了*/
    if(ret != -1){
      break;
    }
  }
  return ret;
}

////////////////////////
/*赤外線ボールの数値を返す*/
////////////////////////
int takiroboF1::getIr(DIRECTION dir)
{
  int ret = 0;
  switch (dir)
  {
  case FRONT:
    ret = _ir1;
    break;
  case RIGHT:
    ret = _ir2;
    break;
  case BACK:
    ret = _ir3;
    break;
  case LEFT:
    ret = _ir4;
    break;
  default:
    ret = 0;
    break;
  }
  return ret;
}

////////////////////////
/*赤外線センサの数値の計算*/
////////////////////////
void takiroboF1::irUpdate(void)
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
    if (digitalRead(IR2) == LOW)
    {
      _ir2++;
    }
    if (digitalRead(IR3) == LOW)
    {
      _ir3++;
    }
    if (digitalRead(IR4) == LOW)
    {
      _ir4++;
    }
  }
}

////////////////////////////////////////
/*方位角の計算　　　　　　　　　　　　　　　　*/
/*メモ：左手座標系なので時計回りを正回転とする*/
////////////////////////////////////////
void takiroboF1::azimUpdate(void)
{
  /*地磁気*/
  if(Mode == HMC5883L)
  {
    /*初期化*/
    Raw_data[0] = 0;
    Raw_data[1] = 0;
    Raw_data[2] = 0;
    int data[2] = {};
    float deg = 0;
    static float pre_deg = 0;
    static long pre_millis = 0;

    //I2C割り込みの許可
    interrupts();

    /*アドレス送信*/
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(HMC5883L_ADDR, 6);

    /*I2Cから数値を取得する*/
    while (Wire.available())
    {
      Raw_data[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
      Raw_data[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
      Raw_data[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    }

    //I2C割り込み禁止
    noInterrupts();

    /*正しく取得できていなかったら*/
    if((Raw_data[0] == 0) || (Raw_data[1] == 0) || (Raw_data[2]  == 0))//エラー処理
    {
      /*DO NOTHING*/
    }
    /*正常に取得できた場合*/
    else
    {
      /*オイラー角度に変換する*/
      data[0] = (Raw_data[0] - Median_x);
      data[1] = (Raw_data[1] - Median_y) * Scale;
      deg=atan2(data[1], data[0]) * 180.0 / PI;
      if (deg<0)
      {
        deg += 360;
      }
    }
    Degree = deg;
  }
  /*ジャイロ*/
  else if(Mode == MPU)
  {
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, Temperature;

    //I2C割り込みの許可
    interrupts();

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Degree = ypr[0] * 180/M_PI;
    }

    //I2C割り込み禁止
    noInterrupts();  
    az_check_flag = false;
  }
  else
  {
    /*設定なし*/
    //DO NOTHING
  }

  
}

////////////////////////////////////////
/*方位角を返す(-180~180)　　　　　　　　　  */
/*メモ：左手座標系なので時計回りを正回転とする*/
////////////////////////////////////////
int takiroboF1::getAzimuth(void)
{
  int ret = 0;
  ret = Degree - Latest_azim;
  while(ret > 179)
  {
    ret -= 360;
  }
  while(ret < -179)
  {
    ret += 360;
  }
  return ret;
}

////////////////////////////////////////////////////
/*HMC5883Lモードにした際にコンパスをキャリブレーションする*/
////////////////////////////////////////////////////
void takiroboF1::calibCompass(void)
{
  int start = millis();
  int finish;
  int now_calib = 0;
  //float Median[2] = {};
  float calib_data[4] = {};
  //float Scale = 0;
  if(Calib == false)
  {
    Serial.println("5秒後にキャリブレーションが開始されます。");
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
      delay(10);
      if (Raw_data[0] < calib_data[0])
      {
        calib_data[0] = Raw_data[0];
        now_calib = 1;
        //Xmin
      }
      if (Raw_data[0] > calib_data[1])
      {
        calib_data[1] = Raw_data[0];
        now_calib = 1;
        //Xmax
      }
      if (Raw_data[1] < calib_data[2])
      {
        calib_data[2] = Raw_data[1];
        now_calib = 1;
        //Ymin
      }
      if (Raw_data[1] > calib_data[3])
      {
        calib_data[3] = Raw_data[1];
        now_calib = 1;
        //Ymax
      }
      finish = millis();
      Serial.println(finish - start);
      if ((finish - start) > 6000 && now_calib == 0)
      {
        Calib = true;
        Serial.println("キャリブレーション終了。");
        Median_x = (calib_data[0] + calib_data[1]) / 2;
        Median_y = (calib_data[2] + calib_data[3]) / 2;
        Scale = ((calib_data[1] - calib_data[0]) / (calib_data[3] - calib_data[2]));
        while(1)
        {
          Serial.print("方位角：");
          Serial.println(getAzimuth());
          if(getBtn() == HIGH)
          {
            unsigned int address = 0x00;
            EEPROM.put(address,Median_x);
            address += sizeof(Median_x);
            EEPROM.put(address,Median_y);
            address += sizeof(Median_y);
            EEPROM.put(address,Scale);
            break;
          }
        }
        break;
      }
      now_calib = 0;
    }
  }
}

//////////////////////
/*ボタンの状態のチェック*/
//////////////////////
void takiroboF1::btnUpdate(void)
{
  static bool last_state = false; 

  if(digitalRead(BTN) != LOW)
  {
    digitalWrite(LED, HIGH);
    Ready_to_start = true;

    if(last_state == false)
    {
      Latest_azim = Degree;
      last_state = true;
    }
  }
  else
  {
    digitalWrite(LED, LOW);
    Ready_to_start = false;
    last_state = false;
  }
}

//////////////////////////////////////
/*ボタン(オルタネートスイッチ)の状態の取得*/
//////////////////////////////////////
bool takiroboF1::getBtn(void)
{
  return Ready_to_start;
}
/////////////////////////////////////////
/*センサモニタ                                              　　　　　　　*/
/*メモ：文字配列を参照渡で返すのでそのデータをSerial.print()に入れると表示できる*/
///////////////////////////////////////////////////////////////////////
void takiroboF1::sensorMonitor(char *str)
{
  char ir_str[50] = "";
  char line_str[50] = "";
  char uss_str[50] = "";
  char azim_str[50] = "";
  char btn_str[50] ="";

  sprintf(ir_str,"IR Sensor F:%03d,R:%03d,B:%03d,L:%03d\r\n",getIr(FRONT),getIr(RIGHT),getIr(BACK),getIr(LEFT));
  sprintf(line_str,"LINE Sensor F:%03d,R:%03d,B:%03d,L:%03d\r\n",getLine(FRONT),getLine(RIGHT),getLine(BACK),getLine(LEFT));
  sprintf(uss_str,"USS Sensor :%03d\r\n",getUSS());
  sprintf(btn_str,"BUTTON Sensor :%01d\r\n",getBtn());
  sprintf(azim_str,"Azimuth Sensor :%03d\r\n",getAzimuth());

  /*strに貼り付け*/
  sprintf(str, "%s%s%s%s%s",ir_str,line_str,uss_str,btn_str,azim_str);
}

////////////////////////////////////////////////////////
/*ロボットのピン設定などを行う(コンストラクタにまとめられないか)*/
////////////////////////////////////////////////////////
void takiroboF1::init(void)
{
  /*ボタンとLED*/
  pinMode(BTN, INPUT);
  pinMode(LED, OUTPUT);

  /*赤外線センサ*/
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  /*超音波センサ*/
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  /*モータ*/
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
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;
  TCCR1A = 0b10100010;//高速PWM TOP値=ICR1
  TCCR1B = 0b00011011;//プリスケーラ64分周
  TCCR2A = 0b10100011;
  TCCR2B = 0b00000100;
  ICR1 = 255;

  /*ラインセンサ*/
  lineSerial.begin(9600);

  switch(Mode)
  {
    /*HMCを利用する場合*/
    case HMC5883L:
      Wire.begin();
      Wire.beginTransmission(HMC5883L_ADDR);
      Wire.write(0x0B);
      Wire.write(0x01);
      Wire.endTransmission();
      Wire.beginTransmission(HMC5883L_ADDR);
      Wire.write(0x09);
      Wire.write(0x1D);
      Wire.endTransmission();

      //キャリブレーション
      Raw_data[0] = 0;
      Raw_data[1] = 0;
      Raw_data[2] = 0;
      while(Ready_to_start == false)
      {
        unsigned int address = 0x00;
        if(Calib == false)
        {
          EEPROM.get(address,Median_x);
          address += sizeof(Median_x);
          EEPROM.get(address,Median_y);
          address += sizeof(Median_y);
          EEPROM.get(address,Scale);
          Ready_to_start = true;
          Serial.print("X:");
          Serial.println(Median_x);
          Serial.print("Y:");
          Serial.println(Median_y);
          Serial.print("S:");
          Serial.println(Scale);
        }
        //キャリブレーション完了合図
        digitalWrite(LED, LOW);
        delay(250);
        digitalWrite(LED, HIGH);
        delay(250);
        digitalWrite(LED, LOW);
        delay(250);
        digitalWrite(LED, HIGH);
        delay(250);
        digitalWrite(LED, LOW);
      }
      break;

    /*MPU6050を利用する場合*/
    case MPU:
      
      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif

      // initialize device
      mpu.initialize();

      // verify connection
      mpu.testConnection();

      // load and configure the DMP
      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
      }
      break;
      
    
    /*何もしない場合*/
    default:
      //Do Nothing
      break;
  }

  /*タイマ割り込み開始*/
  Timer1.initialize(); //2040マイクロ秒周期でタイマ割込みが入る
  Timer1.attachInterrupt(timerISR); 
}
