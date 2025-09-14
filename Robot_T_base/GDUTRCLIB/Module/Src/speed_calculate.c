/**
 * @file speed_calculate.c
 * @author Zhong Yi
 * @brief ����ϵת������ϵ�ٶȼ���ģ��
 * @version 0.1
 */
#include "speed_calculate.h"


void speed_world_calculate(float *vx,float *vy){
float COS,SIN;
	 COS = cos (RealPosData.world_yaw * PI /180);
	 SIN = sin (RealPosData.world_yaw * PI /180);

 // ----------- ��������ϵ�ٶ�ת��Ϊ����������ϵ�ٶ� -----------
    float temp_x = *vx;
    float temp_y = *vy;
    //��ս���;������Ĳ����ӽǲ�ͬ��������ͷģʽҲ��Ҫ����
#if CHANGE_MODE
    *vx = -(temp_x * COS - temp_y * SIN); // ����任��ʽ
    *vy = -(temp_x * SIN + temp_y * COS);
#else 
    
    #if TEST
    *vx = -(temp_x * COS - temp_y * SIN); // ����任��ʽ
    *vy = -(temp_x * SIN + temp_y * COS);
    #else
    *vx = (temp_x * COS - temp_y * SIN); // ����任��ʽ
    *vy = (temp_x * SIN + temp_y * COS);
    #endif
#endif
}

void speed_clock_basket_calculate(float *w)
{
	calc_error();
	*w+=W;
}