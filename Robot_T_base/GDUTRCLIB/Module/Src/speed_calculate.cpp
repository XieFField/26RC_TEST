/**
 * @file speed_calculate.c
 * @author Zhong Yi
 * @brief ����ϵת������ϵ�ٶȼ���ģ��
 * @version 0.1
 */
#include "speed_calculate.h"
#include "ViewCommunication.h"
#include "drive_tim.h"
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
/**
 * @brief ������·���滮��״̬ö��
 */
typedef enum {
    MANHATTAN_IDLE,      // 0: ���л�ȴ���Ŀ��/���о���
    MANHATTAN_MOVING_X,  // 1: ������X���ƶ�
    MANHATTAN_MOVING_Y,  // 2: ������Y���ƶ�
    MANHATTAN_DONE       // 3: ·�����
} ManhattanState_e;

/**
 * @brief ����豹���桿ͨ���޸�Ŀ���ָ����ʵ��������·���滮��
 * @note  �˰汾�ھ���˲�䡰���桱����������꣬��ȷ������·����
 *        ���Ե�ֱ�ߡ����Ǽ�߼���ԡ�����Ժ�³���Ե����㷽����
 * 
 * @param target_x_ptr ָ��Ŀ��X�����ָ��
 * @param target_y_ptr ָ��Ŀ��Y�����ָ��
 * @param current_x ��ǰλ�õ�X����
 * @param current_y ��ǰλ�õ�Y����
 * @param AXIS_TOLERANCE ���ᵽ��Ŀ�������ֵ
 */
void plan_manhattan(float *target_x_ptr, float *target_y_ptr, float current_x, float current_y, float AXIS_TOLERANCE)
{
    static ManhattanState_e manhattan_state = MANHATTAN_IDLE;
    static float final_target_x = -1e9f;
    static float final_target_y = -1e9f;
    static int is_x_path_done = 0;
    static int is_y_path_done = 0;

    // ���¡������ھ���˲����������ı���
    static float latched_x = 0.0f;
    static float latched_y = 0.0f;

    // 1. �������Ŀ���Ƿ�仯������仯����������״̬
    if (fabs(*target_x_ptr - final_target_x) > 1e-6 || fabs(*target_y_ptr - final_target_y) > 1e-6) 
    {
        final_target_x = *target_x_ptr;
        final_target_y = *target_y_ptr;
        manhattan_state = MANHATTAN_IDLE;
        is_x_path_done = 0;
        is_y_path_done = 0;
    }

    // 2. ״̬���߼�
    switch (manhattan_state)
    {
        case MANHATTAN_IDLE:
            // �������ģ�������һ����X����Y������������������
            if (!is_x_path_done && fabs(final_target_x - current_x) > AXIS_TOLERANCE) {
                manhattan_state = MANHATTAN_MOVING_X;
                latched_y = current_y; // ���ؼ������浱ǰ��Y����
            } else if (!is_y_path_done && fabs(final_target_y - current_y) > AXIS_TOLERANCE) {
                manhattan_state = MANHATTAN_MOVING_Y;
                latched_x = current_x; // ���ؼ������浱ǰ��X����
            } else {
                manhattan_state = MANHATTAN_DONE;
            }
            break;

        case MANHATTAN_MOVING_X:
            // Ŀ�����ƶ�X�ᣬY��Ŀ��ʹ������ֵ
            *target_x_ptr = final_target_x;
            *target_y_ptr = latched_y;

            if (fabs(final_target_x - current_x) < AXIS_TOLERANCE) {
                is_x_path_done = 1;
                manhattan_state = MANHATTAN_IDLE;
            }
            break;

        case MANHATTAN_MOVING_Y:
            // Ŀ�����ƶ�Y�ᣬX��Ŀ��ʹ������ֵ
            *target_x_ptr = latched_x;
            *target_y_ptr = final_target_y;

            if (fabs(final_target_y - current_y) < AXIS_TOLERANCE) {
                is_y_path_done = 1;
                manhattan_state = MANHATTAN_IDLE;
            }
            break;

        case MANHATTAN_DONE:
            // ������ɣ��ָ�����Ŀ��
            *target_x_ptr = final_target_x;
            *target_y_ptr = final_target_y;
            break;
    }
}


float dt;
uint32_t last_time;
uint32_t now_time;
void update_timestamp()
{

		if(last_time == 0)
		{
			last_time = Get_SystemTimer();
			return;
		}
		now_time = 	 Get_SystemTimer();

		//overflow
		if(now_time < last_time)
			dt = (float)((now_time + 0xFFFFFFFF) - last_time);  //now_time is 32bit, use 0XFFFFFFFF to avoid overflow
		else
			dt = (float)(now_time - last_time);

		last_time = now_time;

		dt *= 0.000001f;

}
	
PID_T point_X_pid = {0};
PID_T point_Y_pid = {0};

void plan_global_init(void)
{
    pid_param_init(&point_X_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.33f);
    pid_param_init(&point_Y_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.33f);
}




void Plan_Global_Accel(float MAX_ACCEL, float MAX_DECLE, float *global_vx, float *global_vy, int flag) 
{
    // ����б�¼��ٵı���
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
	update_timestamp();
	if(flag)
	{
		// �ٶ�б��/ƽ������ע��Ӽ��ٵķ���Ӧ����ͬ��
		//����·��
		if(*global_vx > 0 && *global_vx >= last_vx)      
		*global_vx = last_vx + MAX_ACCEL*dt;

		else if(*global_vx < 0 && *global_vx <= last_vx)
		*global_vx = last_vx - MAX_ACCEL*dt;	
		
		if(*global_vy > 0 && *global_vy >= last_vy)     
		*global_vy = last_vy + MAX_ACCEL*dt;

		else if(*global_vy < 0 && *global_vy <= last_vy)
		*global_vy = last_vy - MAX_ACCEL*dt;
		
		//����·��,��ע�͵�����Ϊ֧��˲ʱ����
		if(*global_vx > 0 && *global_vx <= last_vx)     
		*global_vx = last_vx - MAX_DECLE*dt;

		else if(*global_vx < 0 && *global_vx >= last_vx)
		*global_vx = last_vx + MAX_DECLE*dt;	
		
		if(*global_vy > 0 && *global_vy <= last_vy)     
		*global_vy = last_vy - MAX_DECLE*dt;

		else if(*global_vy < 0 && *global_vy >= last_vy)
		*global_vy = last_vy + MAX_DECLE*dt;			

	}
	else
	{
		*global_vx = 0.0f;
		*global_vy = 0.0f;
		last_vx = 0.0f;
		last_vy = 0.0f;
	}
	last_vx = *global_vx;
    last_vy = *global_vy;
}

// ״̬ö�ٿ��Լ򻯣�������Ҫר�ŵ�������״̬
typedef enum {
    PLAN_STATE_IDLE,          // 0: ���У��ȴ���Ŀ������
    PLAN_STATE_APPROACH,      // 1: ����ִ��ֱ�߱����ƽ�
    PLAN_STATE_PID_LOCKON,    // 2: ����ִ��PID����
    PLAN_STATE_GOAL_REACHED   // 3: �ѵ���Ŀ��
} PlanState_e;


/**
 * @brief Ϊһ�����ڶ�άƽ���Ϲ滮�ٶȣ����ȫ������ϵ�µ��ٶ�ָ�
 * @note  ���������Ű桿�˰汾�������ٹ滮��Ϊһ��Ŀ��Ԥ��������
 *        ʹ������·���滮���ն���ͳһ��ֱ�߱ƽ���PID�����߼��С�
 * 
 * @param target_x ����Ŀ����X���� (ȫ������ϵ)
 * @param target_y ����Ŀ����Y���� (ȫ������ϵ)
 * @param current_x ��ǰλ�õ�X���� (ȫ������ϵ)
 * @param current_y ��ǰλ�õ�Y���� (ȫ������ϵ)
 * @param global_vx ָ��ȫ��X���ٶ����ֵ��ָ��
 * @param global_vy ָ��ȫ��Y���ٶ����ֵ��ָ��
 */
void plan_global_speed(float target_x, float target_y, float current_x, float current_y, float *global_vx, float *global_vy) 
{
    // --- �ڲ�״̬�;�̬���� ---
    static PlanState_e current_state = PLAN_STATE_IDLE;
    static float desired_vx = 0.0f;
    static float desired_vy = 0.0f;
    static float last_target_x = -1e9f;
    static float last_target_y = -1e9f;
    static float initial_distance_to_goal = 0.0f;
    static int Accel_Flag = 1;
    static int Update_Flag = 1;

    // --- ���Ʋ��� ---
    const float K_P_APPROACH = 0.8f;
    const float GOAL_TOLERANCE = 0.01f;
    const float MAX_LINEAR_SPEED = 1.0f;
    const float MAX_ACCEL = 0.1f;
    const float MAX_DECLE = 0.5f;
    const float LOCKON_PERCENT = 0.02f;
    const float AXIS_TOLERANCE = 0.05f;
    const int g_use_manhattan_planning = 1;			//��Ĭ�����ó������ٹ滮��
    int is_pid_active = 0;

    // --- �ⲿ�¼����� ---
    if (fabs(target_x - last_target_x) > 1e-6 || fabs(target_y - last_target_y) > 1e-6) 
	{
        last_target_x = target_x;
        last_target_y = target_y;
        initial_distance_to_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        current_state = PLAN_STATE_IDLE;
    }

    // --- ���� 1: Ŀ��Ԥ���� (�����پ������) ---
    // ����һ������ЧĿ��㡱���������й滮�����ڴ˵㡣
    float effective_target_x = target_x;
    float effective_target_y = target_y;
     
    if (g_use_manhattan_planning) 
	{
        // ������������٣��������޸ġ���ЧĿ��㡱
        plan_manhattan(&effective_target_x, &effective_target_y, current_x, current_y, AXIS_TOLERANCE);
    }

    // --- ���� 2: ����״̬�� (ͳһ�滮����) ---
    switch (current_state)
    {
        case PLAN_STATE_IDLE:
            Accel_Flag = 0;
            Update_Flag = 0;
            current_state = PLAN_STATE_APPROACH; // ������Σ���һ�����Ǳƽ�
            break;

        case PLAN_STATE_APPROACH:
        {
            // ���ؼ���״̬�л����жϣ�ʼ�ջ��ڵ�������Ŀ��㡱�ľ���
            double distance_to_final_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
            
            if (distance_to_final_goal < GOAL_TOLERANCE) 
			{
                current_state = PLAN_STATE_GOAL_REACHED;
            } 
            // ֻ�е��ӽ�����Ŀ��ʱ�����л���PID����
            else if (distance_to_final_goal < LOCKON_PERCENT * initial_distance_to_goal && initial_distance_to_goal > 0) 
			{
                current_state = PLAN_STATE_PID_LOCKON;
            }

            // �ٶȼ��㣬ʼ�ճ�����ЧĿ��㡱
            double error_x = effective_target_x - current_x;
            double error_y = effective_target_y - current_y;
            desired_vx = error_x * K_P_APPROACH;
            desired_vy = error_y * K_P_APPROACH;
            break;
        }

        case PLAN_STATE_PID_LOCKON:
        {
            double distance_to_final_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
            if (distance_to_final_goal < GOAL_TOLERANCE) 
			{
                current_state = PLAN_STATE_GOAL_REACHED;
            }
            
            // PID���㣬��Զ��������Ŀ��㡱
            is_pid_active = 1;
            *global_vx = pid_calc(&point_X_pid, target_x, current_x);
            *global_vy = pid_calc(&point_Y_pid, target_y, current_y);
            break;
        }

        case PLAN_STATE_GOAL_REACHED:
            desired_vx = 0.0f;
            desired_vy = 0.0f;
            Accel_Flag = 0;
            Update_Flag = 1;
            break;
    }

    // --- ���� 3: ͨ���ٶȺ��� ---
    if (!is_pid_active)
    {
        double total_speed = sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
        if (total_speed > MAX_LINEAR_SPEED) 
        {
            desired_vx = (desired_vx / total_speed) * MAX_LINEAR_SPEED;
            desired_vy = (desired_vy / total_speed) * MAX_LINEAR_SPEED;
        }
        *global_vx = desired_vx;
        *global_vy = desired_vy;
    }

    // --- ���� 4: ����������� ---
    Plan_Global_Accel(MAX_ACCEL, MAX_DECLE, global_vx, global_vy, Accel_Flag);
    Camera_Calibration(Update_Flag);
    Accel_Flag = 1;
}