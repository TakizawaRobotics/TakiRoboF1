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

#ifndef _takiroboF1_H_
#define _takiroboF1_H_
#include <Arduino.h>

/*センサー定義*/
typedef enum //ラインセンサとボールセンサの方向指定で使用
{
	FRONT,
	RIGHT,
	BACK,
	LEFT,
} DIRECTION;

typedef enum //方位角で使用するセンサ
{
	HMC5883L,
	MPU,
    NONE,
} COMPASS;

/*TakiroboF1クラス*/
class takiroboF1
{
public:
    takiroboF1(void);
    takiroboF1(COMPASS mode);
    void init(void);
    void motor(int left, int right);
    void motor(int spd1, int spd2, int spd3);
    void omniControl(float deg, float spd, float yaw);
    int getIr(DIRECTION dir);
    int getLine(DIRECTION dir);
    int getUSS(void);
    bool getBtn(void);
    int getAzimuth(void);
    void sensorMonitor(char *str);
    void calibCompass();

private: 
    static void irUpdate(void);
    static void ussUpdate(void);
    static void btnUpdate(void);
    static void azimUpdate(void);
    static void timerISR(void);
};
#endif