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

PID_T point_X_pid = {0};
PID_T point_Y_pid = {0};

void plan_global_init(void)
{
    pid_param_init(&point_X_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.33f);
    pid_param_init(&point_Y_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.33f);
}


/**
 * @brief Ϊһ�����ڶ�άƽ���Ϲ滮�ٶȣ����ȫ������ϵ�µ��ٶ�ָ�
 * @note  �ú�����������ʽ���ƣ�Զ��ʹ�ñ������ƣ�����(���2%·��)ʹ��PID���㡣
 *        �ٶ� PID ��غ����ͽṹ�����ڱ𴦶��塣
 * 
 * @param target_x Ŀ����X���� (ȫ������ϵ)
 * @param target_y Ŀ����Y���� (ȫ������ϵ)
 * @param current_x ��ǰλ�õ�X���� (ȫ������ϵ)
 * @param current_y ��ǰλ�õ�Y���� (ȫ������ϵ)
 * @param global_vx ָ��ȫ��X���ٶ����ֵ��ָ��
 * @param global_vy ָ��ȫ��Y���ٶ����ֵ��ָ��
 */
void plan_global_speed(float target_x, float target_y, float current_x, float current_y, float *global_vx, float *global_vy) 
{
    // --- ���Ʋ��� ---
    const float K_P_APPROACH = 0.8f;      // �ӽ��׶εı�������
    const float GOAL_TOLERANCE = 0.01f;   // ����Ŀ��ľ�������ֵ (��)
    const float MAX_LINEAR_SPEED = 1.0f;  // �����ٶ� (��/��)
    const float MAX_ACCEL = 0.02f;        // ÿ�ε������ӵ��ٶ����ֵ (����ƽ��)
    const float LOCKON_PERCENT = 0.02;   // �л���PID����ľ���ٷֱ�
    
    // ����б�¼��ٵı���
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;

    // ���ڼ��Ŀ���仯�ʹ洢��ʼ����
    static float last_target_x = -1e9f;
    static float last_target_y = -1e9f;
    static float initial_distance_to_goal = 0.0f;

    // --- �߼���ʼ ---

    // 1. ���Ŀ����Ƿ����仯
    if (fabs(target_x - last_target_x) > 1e-6 || fabs(target_y - last_target_y) > 1e-6) {
        initial_distance_to_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        last_vx = 0.0; // ��Ŀ��Ӿ�ֹ��ʼ
        last_vy = 0.0;
        last_target_x = target_x;
        last_target_y = target_y;
    }

    // 2. ���㵱ǰ��Ŀ��ľ���
    double error_x = target_x - current_x;
    double error_y = target_y - current_y;
    double distance_to_goal = sqrt(error_x * error_x + error_y * error_y);

    // 3. �ж��Ƿ��ѵ���Ŀ���
    if (distance_to_goal < GOAL_TOLERANCE) {
        *global_vx = 0.0;
        *global_vy = 0.0;
        last_vx = 0.0;
        last_vy = 0.0;
        return;
    }

    // 4. ���ݾ���ѡ����Ʋ���
    if (distance_to_goal < LOCKON_PERCENT * initial_distance_to_goal && initial_distance_to_goal > 0) {
        // --- ����B: PID���� ---
        *global_vx = pid_calc(&point_X_pid, target_x, current_x);
        *global_vy = pid_calc(&point_Y_pid, target_y, current_y);
        
        last_vx = *global_vx;
        last_vy = *global_vy;

    } 
    else 
    {
        // --- ����A: �������ƽӽ� ---
        double desired_vx = error_x * K_P_APPROACH;
        double desired_vy = error_y * K_P_APPROACH;

        // ���������ٶ�
        double total_speed = sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
        if (total_speed > MAX_LINEAR_SPEED) 
        {
            desired_vx = (desired_vx / total_speed) * MAX_LINEAR_SPEED;
            desired_vy = (desired_vy / total_speed) * MAX_LINEAR_SPEED;
        }

        // �ٶ�б��/ƽ������
        if (desired_vx > last_vx + MAX_ACCEL) 
            *global_vx = last_vx + MAX_ACCEL;

        else if (desired_vx < last_vx - MAX_ACCEL) 
            *global_vx = last_vx - MAX_ACCEL;

        else 
            *global_vx = desired_vx;

        if (desired_vy > last_vy + MAX_ACCEL) 
            *global_vy = last_vy + MAX_ACCEL;

        else if (desired_vy < last_vy - MAX_ACCEL) 
            *global_vy = last_vy - MAX_ACCEL;

        else 
            *global_vy = desired_vy;
        
        last_vx = *global_vx;
        last_vy = *global_vy;
    }
}