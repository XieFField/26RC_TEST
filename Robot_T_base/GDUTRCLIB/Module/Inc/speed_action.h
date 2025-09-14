#ifndef _SPEED_ACTION_H
#define _SPEED_ACTION_H

#include <stdio.h>
#include "position.h"     // ������������ͷ�ļ�
#include "pid.h"        // ����PID����ͷ�ļ�
#include <math.h>       // ������ѧ������
#define M_PI 3.14159265358979323846f
#define ROBOT_DIAMETER 0.6f  // ���������ֱ������λ����
// ����ʸ���ṹ��
typedef struct {
    float x;
    float y;
} Vector2D;

// ȫ�ֱ�������

extern Vector2D center_point;
extern Vector2D nor_dir;
extern Vector2D tan_dir;
extern float dis_2_center;
extern float center_heading;
extern float nor_speed_x;
extern float nor_speed_y;
// ����µı�������
extern float current_target_radius;      // ��ǰĿ��뾶
extern uint8_t laser_calibration_active; // ����У׼����״̬��־
extern float laser_distance_value;       // ������ֵ
extern float W;
// ��������
void set_multi_radius_rings(void);                                    // ���ö�Բ���뾶����
void laser_calibration_handler(uint8_t status, float distance);      
// ��������
void calc_error(void);
void mode_3(float *robot_vel_x, float *robot_vel_y) ;
void ChassisYaw_Control(float target_yaw,float *w);
void ChassisYawVision_Control(float *w);
// ʸ��������������
Vector2D vector_subtract(Vector2D a, Vector2D b);
Vector2D vector_normalize(Vector2D vec);
float vector_magnitude(Vector2D vec);

void omniYaw_ctrl_T(float *yaw_speed);

void Radar_Control(float target_x, float target_y, float *w);


#define CHANGE_MODE 0 // ֵΪ1ʱ��Ϊ��ս��ͨ�ó���ֵΪ0ʱ��Ϊ������ͨ�ó���
#define TEST 0  //1�������Ϊ�ԼҰ볡����
#endif 