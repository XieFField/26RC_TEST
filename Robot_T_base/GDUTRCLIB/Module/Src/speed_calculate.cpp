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
 
			if (!is_y_path_done && fabs(final_target_y - current_y) >= AXIS_TOLERANCE) 
			{
				manhattan_state = MANHATTAN_MOVING_Y;
				latched_x = current_x; // ���ؼ������浱ǰ��X����
				// Ŀ�����ƶ�Y�ᣬX��Ŀ��ʹ������ֵ				
				*target_x_ptr = latched_x;
				*target_y_ptr = final_target_y;
			} 
			else if (!is_x_path_done && fabs(final_target_x - current_x) >= AXIS_TOLERANCE) 
			{
				manhattan_state = MANHATTAN_MOVING_X;
				latched_y = current_y; // ���ؼ������浱ǰ��Y����
				// Ŀ�����ƶ�X�ᣬY��Ŀ��ʹ������ֵ
				*target_x_ptr = final_target_x;
				*target_y_ptr = latched_y;
			}
			else 
			{
                manhattan_state = MANHATTAN_DONE;
            }
            break;

        case MANHATTAN_MOVING_X:
			/* ���ƶ������У�����ǿ���趨Ŀ��㣬ȷ��·���ȶ�������Ҫע�⣬
			�������ܻ�����߼���ͻ���⣬ά��ʱ�м��������ظ���ֵ�Ķ���*/
			// Ŀ�����ƶ�X�ᣬY��Ŀ��ʹ������ֵ
			*target_x_ptr = final_target_x;
			*target_y_ptr = latched_y;			
            if (fabs(final_target_x - current_x) < AXIS_TOLERANCE) 
			{
                is_x_path_done = 1;
                manhattan_state = MANHATTAN_IDLE;
            }
            break;

        case MANHATTAN_MOVING_Y:
			/* ���ƶ������У�����ǿ���趨Ŀ��㣬ȷ��·���ȶ�������Ҫע�⣬
			�������ܻ�����߼���ͻ���⣬ά��ʱ�м��������ظ���ֵ�Ķ���*/
			// Ŀ�����ƶ�Y�ᣬX��Ŀ��ʹ������ֵ
			*target_x_ptr = latched_x;
			*target_y_ptr = final_target_y;
            if (fabs(final_target_y - current_y) < AXIS_TOLERANCE) 
			{
                is_y_path_done = 1;
                manhattan_state = MANHATTAN_IDLE;
            }
            break;

        case MANHATTAN_DONE:
            // ������ɣ��ָ�����Ŀ��
            *target_x_ptr = final_target_x;
            *target_y_ptr = final_target_y;
		    manhattan_state = MANHATTAN_IDLE;
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
    pid_param_init(&point_X_pid, PID_Position, 1.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.05f);
    pid_param_init(&point_Y_pid, PID_Position, 1.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.05f);
}




void Plan_Global_Accel(float MAX_ACCEL, float *global_vx, float *global_vy, int flag) 
{
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
    update_timestamp();

    if(flag)
    {
		//ʹ�ü��ٶȿ��Ƶ����ٶ�(��ʹ�ü��ٶ��޷���ֱ�Ӽ�ͣ��
		/*------------------------------------------------------------------------------*/
		if(*global_vx > 0 && *global_vx >= last_vx)      //���ٶ��޷�
		*global_vx = last_vx + 2.5*MAX_ACCEL*dt;

		else if(*global_vx < 0 && *global_vx <= last_vx)
		*global_vx = last_vx - 2.5*MAX_ACCEL*dt;
		/*------------------------------------------------------------------------------*/
		else
		{;}
		/*------------------------------------------------------------------------------*/

		if(*global_vy > 0 && *global_vy >= last_vy)       //���ٶ��޷�
		*global_vy = last_vy + 2.5 * MAX_ACCEL*dt;

		else if(*global_vy < 0 && *global_vy <= last_vy)
		*global_vy = last_vy - 2.5 * MAX_ACCEL*dt;
		/*------------------------------------------------------------------------------*/
		else
		{;}
    }
    else
    {
		//�����κδ���
    }
	

	last_vx = *global_vx;
	last_vy = *global_vy;
}

/**
 * @brief ����һά�����ٶȹ滮�µ�Ŀ���ٶ�
 * @param total_distance        �����˶����ܾ���
 * @param distance_to_goal      ��ǰ�����յ�ľ���
 * @param max_speed             ���������ٶ�
 * @param max_accel             ����������ٶ�
 * @return float                ��ǰʱ�̵�Ŀ���ٶ�
 */
float calculate_trapezoidal_speed(float total_distance, float distance_to_goal, float max_speed, float max_accel)
{
    // 1. ���������ٶȼ��ٵ�0����ľ��루ɲ�����룩
    float braking_distance = (max_speed * max_speed) / (2.0f * max_accel);

    // 2. �ж�·���ǡ���·���������Σ����ǡ���·�����������Σ�
    if (total_distance > 2.0f * braking_distance)
    {
        // --- ����A: ��·�������Դﵽ����ٶȣ���׼���Σ�---
        float distance_traveled = total_distance - distance_to_goal;
        
        if (distance_traveled < braking_distance)
        {
            // (1) ���ٶ�: �ٶ�����ʻ�������
            // v^2 = 2*a*d => v = sqrt(2*a*d)
            return sqrtf(2.0f * max_accel * distance_traveled);
        }
        else if (distance_to_goal < braking_distance)
        {
            // (3) ���ٶ�: �ٶ���ʣ��������
            // v^2 = 2*a*d => v = sqrt(2*a*d)
            return sqrtf(2.0f * max_accel * distance_to_goal);
        }
        else
        {
            // (2) ���ٶ�
            return max_speed;
        }
    }
    else
    {
        // --- ����B: ��·�����޷��ﵽ����ٶȣ������Σ�---
        float halfway_distance = total_distance / 2.0f;
        float distance_traveled = total_distance - distance_to_goal;

        if (distance_traveled < halfway_distance)
        {
            // (1) ���ٶ�
            return sqrtf(2.0f * max_accel * distance_traveled);
        }
        else
        {
            // (2) ���ٶ�
            return sqrtf(2.0f * max_accel * distance_to_goal);
        }
    }
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
	// ���ڼ�¼ÿһ����Ч·������ʼ����
    static float effective_path_total_distance = 0.0f;
    static float last_effective_target_x = -1e9f;
    static float last_effective_target_y = -1e9f;
	//��ʼ������
    int Update_Flag = 1;
	int is_pid_active = 0;
	
    // --- ���Ʋ��� ---
    const float K_P_APPROACH = 0.8f;
    const float GOAL_TOLERANCE = 0.04f;
    const float MAX_LINEAR_SPEED = 1.0f;
    const float MAX_ACCEL = 1.0f;
    const float LOCKON_PERCENT = 0.02f;
    const float AXIS_TOLERANCE = 0.05f;
    const int g_use_manhattan_planning = 1;			//��Ĭ�����ó������ٹ滮��
    

    // --- �ⲿ�¼����� ---
    // ʹ�þ������ж��Ƿ�Ϊ��Ŀ�꣬�������ü�С�ĸ������ݲ�
    double distance_between_targets = sqrt(pow(target_x - last_target_x, 2) + pow(target_y - last_target_y, 2));

    if (distance_between_targets > GOAL_TOLERANCE) 
    {
        last_target_x = target_x;
        last_target_y = target_y;
        initial_distance_to_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        current_state = PLAN_STATE_IDLE;
//        plan_global_init();
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
    // �����ЧĿ����Ƿ�仯������仯������¸ö�·�����ܾ���
    if (fabs(effective_target_x - last_effective_target_x) > 1e-6 || fabs(effective_target_y - last_effective_target_y) > 1e-6)
    {
        last_effective_target_x = effective_target_x;
        last_effective_target_y = effective_target_y;
        effective_path_total_distance = sqrt(pow(effective_target_x - current_x, 2) + pow(effective_target_y - current_y, 2));
    }
    // --- ���� 2: ����״̬�� (ͳһ�滮����) ---
    switch (current_state)
    {
        case PLAN_STATE_IDLE:
			desired_vx = 0;
			desired_vy = 0;
            Accel_Flag = 0;
            Update_Flag = 0;
            current_state = PLAN_STATE_APPROACH; // ������Σ���һ�����Ǳƽ�
            break;

        case PLAN_STATE_APPROACH:
        {
			Accel_Flag = 1;
			Update_Flag = 0; 
            // ״̬�л����жϣ�ʼ�ջ��ڵ�������Ŀ��㡱�ľ���
            double distance_to_final_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
           
            // ֻ�е��ӽ�����Ŀ��ʱ�����л���PID����
            if (distance_to_final_goal < LOCKON_PERCENT * initial_distance_to_goal && initial_distance_to_goal > 0) 
			{
                current_state = PLAN_STATE_PID_LOCKON;
            }

            // �������޸ġ�ʹ�������ٶȹ滮ȡ��P������
            // 1. ���㵱ǰ������ЧĿ��㡱�ľ���ͷ���
            double error_x = effective_target_x - current_x;
            double error_y = effective_target_y - current_y;
            double distance_to_effective_goal = sqrt(error_x * error_x + error_y * error_y);

            // 2. �������ι滮�����������ǰӦ���еġ������ٶȡ�
            // 2. �������������������ι滮��ʱ��ʹ�õ�ǰ��Ч·�����ܾ���
            float target_scalar_speed = calculate_trapezoidal_speed(effective_path_total_distance, 
																	distance_to_effective_goal, 
																	MAX_LINEAR_SPEED, MAX_ACCEL);

            // 3. �������ٶȷֽ⵽X��Y����
            if (distance_to_effective_goal > 1e-6) // ��ֹ������
            {
                float dir_x = error_x / distance_to_effective_goal;
                float dir_y = error_y / distance_to_effective_goal;
                desired_vx = dir_x * target_scalar_speed;
                desired_vy = dir_y * target_scalar_speed;
            }
            else
            {
                desired_vx = 0.0f;
                desired_vy = 0.0f;
            }
            break;
        }

        case PLAN_STATE_PID_LOCKON:
        {
            double distance_to_final_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
            Accel_Flag = 1;
            // PID���㣬��Զ��������Ŀ��㡱
            is_pid_active = 1;
            *global_vx = pid_calc(&point_X_pid, target_x, current_x);
            *global_vy = pid_calc(&point_Y_pid, target_y, current_y);
            break;
        }
    }

    // --- ���� 3: ͨ���ٶȺ���(��PID����� ---
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
  
//    Camera_Calibration(Update_Flag);
	Plan_Global_Accel(MAX_ACCEL, global_vx, global_vy, Accel_Flag);

}