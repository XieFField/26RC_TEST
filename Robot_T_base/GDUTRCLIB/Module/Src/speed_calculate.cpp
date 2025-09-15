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

/**
 * @brief 为一个点在二维平面上规划速度，输出全局坐标系下的速度指令。
 * @note  该函数采用两段式控制：远处使用比例控制，近处(最后2%路程)使用PID锁点。
 *        假定 PID 相关函数和结构体已在别处定义。
 * 
 * @param target_x 目标点的X坐标 (全局坐标系)
 * @param target_y 目标点的Y坐标 (全局坐标系)
 * @param current_x 当前位置的X坐标 (全局坐标系)
 * @param current_y 当前位置的Y坐标 (全局坐标系)
 * @param global_vx 指向全局X轴速度输出值的指针
 * @param global_vy 指向全局Y轴速度输出值的指针
 */
void plan_global_speed(float target_x, float target_y, float current_x, float current_y, float *global_vx, float *global_vy) 
{
	static float desired_vx = 0;
	static float desired_vy = 0;
    // --- 控制参数 ---
    const float K_P_APPROACH = 0.8f;      // 接近阶段的比例增益
    const float GOAL_TOLERANCE = 0.01f;   // 到达目标的距离容忍值 (米)
    const float MAX_LINEAR_SPEED = 1.0f;  // 最大合速度 (米/秒)
    const float MAX_ACCEL = 0.1f;        // 每次调用增加的速度最大值 (用于平滑)
	const float MAX_DECLE = 0.5f;        // 每次调用增加的速度最大值 (用于平滑)
    const float LOCKON_PERCENT = 0.02;   // 切换到PID锁点的距离百分比
    static int Accel_Flag = 1;					//平滑速度规划器
	static int Update_Flag = 1;					//目标点更新开关

    // 用于检测目标点变化和存储初始距离
    static float last_target_x = -1e9f;
    static float last_target_y = -1e9f;
    static float initial_distance_to_goal = 0.0f;

    // --- 逻辑开始 ---
	
    // 1. 检测目标点是否发生变化
    if (fabs(target_x - last_target_x) > 1e-6 || fabs(target_y - last_target_y) > 1e-6) {
        initial_distance_to_goal = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        Accel_Flag = 0;
		Update_Flag = 0;
        last_target_x = target_x;
        last_target_y = target_y;
    }

    // 2. 计算当前到目标的距离
    double error_x = target_x - current_x;
    double error_y = target_y - current_y;
    double distance_to_goal = sqrt(error_x * error_x + error_y * error_y);

    // 3. 判断是否已到达目标点
    if (distance_to_goal < GOAL_TOLERANCE) {
        Accel_Flag = 0;
		Update_Flag = 1;
    }

    // 4. 根据距离选择控制策略
    if (distance_to_goal < LOCKON_PERCENT * initial_distance_to_goal && initial_distance_to_goal > 0) {
        // --- 策略B: PID锁点 ---
        *global_vx = pid_calc(&point_X_pid, target_x, current_x);
        *global_vy = pid_calc(&point_Y_pid, target_y, current_y);
    } 
    else 
    {
        // --- 策略A: 比例控制接近 ---
        desired_vx = error_x * K_P_APPROACH;
        desired_vy = error_y * K_P_APPROACH;

        // 限制最大合速度
        double total_speed = sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
        if (total_speed > MAX_LINEAR_SPEED) 
        {
            desired_vx = (desired_vx / total_speed) * MAX_LINEAR_SPEED;
            desired_vy = (desired_vy / total_speed) * MAX_LINEAR_SPEED;
        }
		*global_vx = desired_vx;
		*global_vy = desired_vy;
    }
	//平滑速度限幅器
	Plan_Global_Accel(MAX_ACCEL,MAX_DECLE, global_vx, global_vy, Accel_Flag);
	//目标点更新器
	Camera_Calibration(Update_Flag);
	//重置开关状态
	Accel_Flag = 1;

}