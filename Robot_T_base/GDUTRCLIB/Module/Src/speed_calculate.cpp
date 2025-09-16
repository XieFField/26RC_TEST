/**
 * @file speed_calculate.c
 * @author Zhong Yi
 * @brief 世界系转机器人系速度计算模块
 * @version 0.1
 */
#include "speed_calculate.h"
#include "ViewCommunication.h"
#include "drive_tim.h"
void speed_world_calculate(float *vx,float *vy){
float COS,SIN;
	 COS = cos (RealPosData.world_yaw * PI /180);
	 SIN = sin (RealPosData.world_yaw * PI /180);

 // ----------- 世界坐标系速度转换为机器人坐标系速度 -----------
    float temp_x = *vx;
    float temp_y = *vy;
    //挑战赛和竞技赛的操作视角不同，所以无头模式也需要换向
#if CHANGE_MODE
    *vx = -(temp_x * COS - temp_y * SIN); // 坐标变换公式
    *vy = -(temp_x * SIN + temp_y * COS);
#else 
    
    #if TEST
    *vx = -(temp_x * COS - temp_y * SIN); // 坐标变换公式
    *vy = -(temp_x * SIN + temp_y * COS);
    #else
    *vx = (temp_x * COS - temp_y * SIN); // 坐标变换公式
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
 * @brief 曼哈顿路径规划的状态枚举
 */
typedef enum {
    MANHATTAN_IDLE,      // 0: 空闲或等待新目标/进行决策
    MANHATTAN_MOVING_X,  // 1: 正在沿X轴移动
    MANHATTAN_MOVING_Y,  // 2: 正在沿Y轴移动
    MANHATTAN_DONE       // 3: 路径完成
} ManhattanState_e;

/**
 * @brief 【返璞归真版】通过修改目标点指针来实现曼哈顿路径规划。
 * @note  此版本在决策瞬间“锁存”正交轴的坐标，以确保后续路径是
 *        绝对的直线。这是兼具简洁性、灵活性和鲁棒性的优秀方案。
 * 
 * @param target_x_ptr 指向目标X坐标的指针
 * @param target_y_ptr 指向目标Y坐标的指针
 * @param current_x 当前位置的X坐标
 * @param current_y 当前位置的Y坐标
 * @param AXIS_TOLERANCE 单轴到达目标的容忍值
 */
void plan_manhattan(float *target_x_ptr, float *target_y_ptr, float current_x, float current_y, float AXIS_TOLERANCE)
{
    static ManhattanState_e manhattan_state = MANHATTAN_IDLE;
    static float final_target_x = -1e9f;
    static float final_target_y = -1e9f;
    static int is_x_path_done = 0;
    static int is_y_path_done = 0;

    // 【新】用于在决策瞬间锁存坐标的变量
    static float latched_x = 0.0f;
    static float latched_y = 0.0f;

    // 1. 检测最终目标是否变化，如果变化则重置所有状态
    if (fabs(*target_x_ptr - final_target_x) > 1e-6 || fabs(*target_y_ptr - final_target_y) > 1e-6) 
    {
        final_target_x = *target_x_ptr;
        final_target_y = *target_y_ptr;
        manhattan_state = MANHATTAN_IDLE;
        is_x_path_done = 0;
        is_y_path_done = 0;
    }

    // 2. 状态机逻辑
    switch (manhattan_state)
    {
        case MANHATTAN_IDLE:
            // 决策中心：决定下一步走X还是Y，并锁存正交轴坐标
            if (!is_x_path_done && fabs(final_target_x - current_x) > AXIS_TOLERANCE) {
                manhattan_state = MANHATTAN_MOVING_X;
                latched_y = current_y; // 【关键】锁存当前的Y坐标
            } else if (!is_y_path_done && fabs(final_target_y - current_y) > AXIS_TOLERANCE) {
                manhattan_state = MANHATTAN_MOVING_Y;
                latched_x = current_x; // 【关键】锁存当前的X坐标
            } else {
                manhattan_state = MANHATTAN_DONE;
            }
            break;

        case MANHATTAN_MOVING_X:
            // 目标是移动X轴，Y轴目标使用锁存值
            *target_x_ptr = final_target_x;
            *target_y_ptr = latched_y;

            if (fabs(final_target_x - current_x) < AXIS_TOLERANCE) {
                is_x_path_done = 1;
                manhattan_state = MANHATTAN_IDLE;
            }
            break;

        case MANHATTAN_MOVING_Y:
            // 目标是移动Y轴，X轴目标使用锁存值
            *target_x_ptr = latched_x;
            *target_y_ptr = final_target_y;

            if (fabs(final_target_y - current_y) < AXIS_TOLERANCE) {
                is_y_path_done = 1;
                manhattan_state = MANHATTAN_IDLE;
            }
            break;

        case MANHATTAN_DONE:
            // 任务完成，恢复最终目标
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
    // 用于斜坡加速的变量
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
	update_timestamp();
	if(flag)
	{
		// 速度斜坡/平滑处理（注意加减速的符号应该相同）
		//加速路段
		if(*global_vx > 0 && *global_vx >= last_vx)      
		*global_vx = last_vx + MAX_ACCEL*dt;

		else if(*global_vx < 0 && *global_vx <= last_vx)
		*global_vx = last_vx - MAX_ACCEL*dt;	
		
		if(*global_vy > 0 && *global_vy >= last_vy)     
		*global_vy = last_vy + MAX_ACCEL*dt;

		else if(*global_vy < 0 && *global_vy <= last_vy)
		*global_vy = last_vy - MAX_ACCEL*dt;
		
		//减速路段,先注释掉，因为支持瞬时减速
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

// 状态枚举可以简化，不再需要专门的曼哈顿状态
typedef enum {
    PLAN_STATE_IDLE,          // 0: 空闲，等待新目标或决策
    PLAN_STATE_APPROACH,      // 1: 正在执行直线比例逼近
    PLAN_STATE_PID_LOCKON,    // 2: 正在执行PID锁点
    PLAN_STATE_GOAL_REACHED   // 3: 已到达目标
} PlanState_e;


/**
 * @brief 为一个点在二维平面上规划速度，输出全局坐标系下的速度指令。
 * @note  【最终优雅版】此版本将曼哈顿规划视为一个目标预处理器，
 *        使得所有路径规划最终都能统一到直线逼近和PID锁点逻辑中。
 * 
 * @param target_x 最终目标点的X坐标 (全局坐标系)
 * @param target_y 最终目标点的Y坐标 (全局坐标系)
 * @param current_x 当前位置的X坐标 (全局坐标系)
 * @param current_y 当前位置的Y坐标 (全局坐标系)
 * @param global_vx 指向全局X轴速度输出值的指针
 * @param global_vy 指向全局Y轴速度输出值的指针
 */
void plan_global_speed(float target_x, float target_y, float current_x, float current_y, float *global_vx, float *global_vy) 
{
    // --- 内部状态和静态变量 ---
    static PlanState_e current_state = PLAN_STATE_IDLE;
    static float desired_vx = 0.0f;
    static float desired_vy = 0.0f;
    static float last_target_x = -1e9f;
    static float last_target_y = -1e9f;
    static float initial_distance_to_goal = 0.0f;
    static int Accel_Flag = 1;
    static int Update_Flag = 1;

    // --- 控制参数 ---
    const float K_P_APPROACH = 0.8f;
    const float GOAL_TOLERANCE = 0.01f;
    const float MAX_LINEAR_SPEED = 1.0f;
    const float MAX_ACCEL = 0.1f;
    const float MAX_DECLE = 0.5f;
    const float LOCKON_PERCENT = 0.02f;
    const float AXIS_TOLERANCE = 0.05f;
    const int g_use_manhattan_planning = 1;			//先默认设置成曼哈顿规划吧
    int is_pid_active = 0;

    // --- 外部事件处理 ---
    if (fabs(target_x - last_target_x) > 1e-6 || fabs(target_y - last_target_y) > 1e-6) 
	{
        last_target_x = target_x;
        last_target_y = target_y;
        initial_distance_to_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        current_state = PLAN_STATE_IDLE;
    }

    // --- 功能 1: 目标预处理 (曼哈顿决策入口) ---
    // 创建一个“有效目标点”，后续所有规划都基于此点。
    float effective_target_x = target_x;
    float effective_target_y = target_y;
     
    if (g_use_manhattan_planning) 
	{
        // 如果启用曼哈顿，则让它修改“有效目标点”
        plan_manhattan(&effective_target_x, &effective_target_y, current_x, current_y, AXIS_TOLERANCE);
    }

    // --- 功能 2: 核心状态机 (统一规划流程) ---
    switch (current_state)
    {
        case PLAN_STATE_IDLE:
            Accel_Flag = 0;
            Update_Flag = 0;
            current_state = PLAN_STATE_APPROACH; // 无论如何，下一步都是逼近
            break;

        case PLAN_STATE_APPROACH:
        {
            // 【关键】状态切换的判断，始终基于到“最终目标点”的距离
            double distance_to_final_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
            
            if (distance_to_final_goal < GOAL_TOLERANCE) 
			{
                current_state = PLAN_STATE_GOAL_REACHED;
            } 
            // 只有当接近最终目标时，才切换到PID锁点
            else if (distance_to_final_goal < LOCKON_PERCENT * initial_distance_to_goal && initial_distance_to_goal > 0) 
			{
                current_state = PLAN_STATE_PID_LOCKON;
            }

            // 速度计算，始终朝向“有效目标点”
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
            
            // PID锁点，永远锁向“最终目标点”
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

    // --- 功能 3: 通用速度后处理 ---
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

    // --- 功能 4: 最终输出处理 ---
    Plan_Global_Accel(MAX_ACCEL, MAX_DECLE, global_vx, global_vy, Accel_Flag);
    Camera_Calibration(Update_Flag);
    Accel_Flag = 1;
}