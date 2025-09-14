#ifndef _SPEED_CALCULATE_H
#define _SPEED_CALCULATE_H

#include <stdio.h>
#include "position.h"     // ������������ͷ�ļ�
#include "pid.h"        // ����PID����ͷ�ļ�
#include "speed_action.h"
#include <math.h>       // ������ѧ������
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // ���������ֱ������λ����
void speed_world_calculate(float *vx,float *vy);
void speed_clock_basket_calculate(float *w);


#endif 