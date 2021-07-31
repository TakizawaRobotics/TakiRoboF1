#ifndef _takiroboF1_H_
#define _takiroboF1_H_
#include "arduino.h"
class takiroboF1
{
public:
    takiroboF1(float median_x, float median_y, float scale);
    takiroboF1();
    void init();
    void calib_compass();
    void irUpdate();
    void lineUpdate();
    int getIr(int num);
    int getLine(int num);
    float getStartingAzimuth();
    float getAzimuth();
    double getUSS();
    void motor(double spd1, double spd2, double spd3);

private:
    float starting_position_deg = 0;
    int _ir1 = 0;
    int _ir2 = 0;
    int _ir3 = 0;
    int _ir4 = 0;
    float MEDIAN_x;
    float MEDIAN_y;
    float SCALE;
    int raw_data[3] = {};
    int line[4];

};
#endif