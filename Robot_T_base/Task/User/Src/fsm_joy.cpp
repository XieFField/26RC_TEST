/**
 * @file fsm_joy.cpp
 * @author Wujia
 * @brief 遥控状态机
 * @version 0.1
 * @date 2025-05-23
 * 
 * @version Final
 * @date    2025-8-13
 * @brief   删去一些先前的测试语句
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"
#include "speed_calculate.h"
#include "lora.h"
#include "LED.h"
#include "speed_action.h"
#include "ViewCommunication.h"

#define LASER_CALIBRA_YAW   0.0f   //激光重定位时候车锁定的yaw轴数值
int test = 0;
void Air_Joy_Task(void *pvParameters)
{
    //LED_Init();
    static CONTROL_T ctrl;
    static uint8_t SWD_D_state = 0; //0为UP 1为DOWN
    fsm_joy_timer.fsm_joy_timer_started = false;
    fsm_joy_timer.fsm_joy_start_tick = 0;

    static bool change_key = false;

    for(;;)
    {
        if(air_joy.RIGHT_X>1450&&air_joy.RIGHT_X<1550)
            air_joy.RIGHT_X = 1500;
        if(air_joy.RIGHT_Y>1450&&air_joy.RIGHT_Y<1550)  
            air_joy.RIGHT_Y = 1500;
        //遥杆消抖

		if(air_joy.LEFT_X>1450&&air_joy.LEFT_X<1550&&air_joy.LEFT_Y>1450&&air_joy.LEFT_Y<1550)
		{
			if (!fsm_joy_timer.fsm_joy_timer_started)
            {
                fsm_joy_timer.fsm_joy_start_tick = xTaskGetTickCount();
                fsm_joy_timer.fsm_joy_timer_started = true;
            }
			if (xTaskGetTickCount() - fsm_joy_timer.fsm_joy_start_tick >= pdMS_TO_TICKS(50))
            {
				air_joy.LEFT_X = 1500;
                air_joy.LEFT_Y = 1500;
            }
		}
        else
        {
			// 重置定时器状态
            fsm_joy_timer.fsm_joy_timer_started = false;
            fsm_joy_timer.fsm_joy_start_tick = 0;
		}
        
        //遥控器启动判断
        if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
        {
            // static ReceiveRealData_S Pos_Now = {0};
            // xQueueReceive(VISION_TO_REAL_Port, &Pos_Now, 0);
            if(_tool_Abs(air_joy.SWB - 1000) > 400)
            {                
                
                /*======================================================*/
                if(_tool_Abs(air_joy.SWB - 1500) < 50)
                {
                    ctrl.robot_crtl = BALL_MODE;  
                    ctrl.twist.linear.y = -(air_joy.LEFT_Y - 1500)/500.0 * 3.7;
                    ctrl.twist.linear.x = -(air_joy.LEFT_X - 1500)/500.0 * 3.7;
                    ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 2.5;

                    ctrl.twist.pitch.column = (air_joy.RIGHT_Y - 1500)/500.0 * 2;
                    speed_world_calculate(&ctrl.twist.linear.x,&ctrl.twist.linear.y);
					Plan_Global_Accel(1,8,&ctrl.twist.linear.x,&ctrl.twist.linear.y,1);

                    if(_tool_Abs(air_joy.SWA - 1000) < 50) //SWA UP
                        ctrl.chassis_ctrl = CHASSIS_COM_MODE;   //普通移动

                    else if(_tool_Abs(air_joy.SWA - 2000) < 50) //SWA DOWN
                    { 
                       

                        //ChassisYaw_Control(0.45/M_PI*180, Pos_Now.yaw*180/M_PI, &ctrl.twist.linear.z);//锁角
                        ctrl.chassis_ctrl = CHASSIS_CAMERA_CALIBRA; //相机标定模式
                        /**
                         * @brief 相机标定
                         *        切换到SWA DOWN的时候
                         *        先读取当前的SWD状态(只有UP 和 DOWN两种状态)
                         *        然后若改变SWD的状态，则发送相机标定指令
                         *        若再次改变SWD的状态，则发送相机结束标定指令
                         */
                        if(!change_key) //每次进入SWA DOWM模式都会初始化一次键位
                        {
                            if(_tool_Abs(air_joy.SWD - 1000)<50)
                                SWD_D_state = 0; //UP
                            else if(_tool_Abs(air_joy.SWD - 2000)<50)
                                SWD_D_state = 1; //DOWN
                            change_key = true;
                            test = 3;
                        }
                        if(change_key)
                        {
                            if(SWD_D_state == 0 && _tool_Abs(air_joy.SWD - 2000) <50) //UP -> DOWN
                            {
                                SWD_D_state = 1;
                                test = 1;
                                for(int i =0; i < 100; i++) Camera_Calibration(1); //开始标定
                                   
                                change_key = false;
                            }
                            else if(SWD_D_state == 1 && _tool_Abs(air_joy.SWD - 1000)<50) //DOWN -> UP
                            {
                                SWD_D_state = 0;
                                for(int i =0; i < 100; i++)Camera_Calibration(0); //结束标定
                                    
                                test = 2;
                                change_key = false;
                            }
                        }
                        
                    }
                }
                /*-========================================================-*/

                else if(_tool_Abs(air_joy.SWB - 2000) < 50)
                {
                    change_key = false;
                    ctrl.robot_crtl = ROAD_AUTO_MODE;   //自动模式
                    if(_tool_Abs(air_joy.SWA - 1000) < 50)
                        ctrl.chassis_ctrl = CHASSIS_LOW_MODE;
                    else if(_tool_Abs(air_joy.SWA - 2000) < 50)
                        ctrl.chassis_ctrl = CHASSIS_LOCK_TARGET;    //底盘锁定篮筐                   
                } 
            }
            else//所有机构全部关闭
            {
                ctrl.robot_crtl = OFF_MODE;
                ctrl.chassis_ctrl = CHASSIS_OFF;
            }
            xQueueSend(Chassia_Port, &ctrl, 0);
        }
        else
        {
            ctrl.twist = {0};
        }

        osDelay(1);
    }
}
