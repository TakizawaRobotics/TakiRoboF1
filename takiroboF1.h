/*======================================================================
Project Name    : takiroboF1
File Name       : takiroboF1.h
Encoding        : utf-8
Creation Date   : c++
author          : Takumi Yoneda, Hayato Tajiri
update date     : 2021/11/13 
 
Copyright © <2021> TakizawaRoboticsLLC. All rights reserved.
 
This source code or any portion thereof must not be  
reproduced or used in any manner whatsoever.
======================================================================*/

#ifndef _takiroboF1_H_
#define _takiroboF1_H_
#include "arduino.h"
class takiroboF1
{
public:
    takiroboF1();
    void init();
    void motor(double spd1, double spd2, double spd3);
    void mtDeg(int deg, int spd);
    void irUpdate();
    int getIr(int num);
    int getLine(int num);
    double getUSS();
    double getAzimuth();
    static void interrupt();

private:
    void azimUpdate();
    void calib_compass();
    volatile int _ir1;
    volatile int _ir2;
    volatile int _ir3;
    volatile int _ir4;
    volatile int calib;
    double firstAzim;
    double degree;
    double MEDIAN_x;
    double MEDIAN_y;
    double SCALE;
    int raw_data[3] = {};
};
#endif