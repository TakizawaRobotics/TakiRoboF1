#ifndef _takiroboF1_H_
#define _takiroboF1_H_
#include "arduino.h"
class takiroboF1
{
public:
    takiroboF1(float MedianX, float MedianY, float Scale);
    takiroboF1();
    void init();
    void calibCompass();
    void irUpdate();
    void lineUpdate();
    int getIr(int num);
    int getLine(int num);
    float getStartAzim();
    float getAzim();
    double getUSS();
    void motor(double spd1, double spd2, double spd3);

private:
    double starting_position_deg = 0;
    int _ir1;
    int _ir2;
    int _ir3;
    int _ir4;
    float medianX;
    float medianY;
    float scale;
    int rawData[3] = {};
    int line[4];

};
#endif