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


#define LASER_CALIBRA_YAW   0.0f   //激光重定位时候车锁定的yaw轴数值

void Air_Joy_Task(void *pvParameters)
{
    //LED_Init();
    static CONTROL_T ctrl;
    static SHOOT_JUDGEMENT_E shoot_judge = POSITION;
    fsm_joy_timer.fsm_joy_timer_started = false;
    fsm_joy_timer.fsm_joy_start_tick = 0;

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

        xQueueReceive(Shoot_Judge_Port, &shoot_judge, pdTRUE);
        
        //遥控器启动判断
        if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
        {
                
            if(_tool_Abs(air_joy.SWB - 1000) > 400)
            {                
                     
                /*======================================================*/
                if(_tool_Abs(air_joy.SWB - 1500) < 50)//接球模式
                {
                    ctrl.robot_crtl = BALL_MODE;  
                    
                     
                    
                    ctrl.twist.linear.y = -(air_joy.LEFT_Y - 1500)/500.0 * 3.7;
                    ctrl.twist.linear.x = -(air_joy.LEFT_X - 1500)/500.0 * 3.7;
                    ctrl.twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 2.5;

                    ctrl.twist.pitch.column = (air_joy.RIGHT_Y - 1500)/500.0 * 2;
                    speed_world_calculate(&ctrl.twist.linear.x,&ctrl.twist.linear.y);
                    #if CHANGE_MODE
                            ctrl.dribble_ctrl = DRIBBLE_OFF;
                            ctrl.pitch_ctrl = PITCH_LOCK_MODE;
                    #else

                    static CONTROL_T ctrl_last;

                    ctrl_last = ctrl;

                    #endif //运球挑战赛/竞技赛开关


                    

                    if(_tool_Abs(air_joy.SWA - 1000) < 50) //SWA UP
                    {
                        
                        if(_tool_Abs(air_joy.SWD - 1000) < 50)
                        {
                            ctrl.laser_ctrl = LASER_CALIBRA_OFF;
                            #if CHANGE_MODE
                            ctrl.chassis_ctrl = CHASSIS_COM_MODE;   //普通移动
                            #endif
                        }
                        else if(_tool_Abs(air_joy.SWD - 2000) < 50 && _tool_Abs(air_joy.SWC - 1000) < 50 )
                        {
                            ChassisYaw_Control(LASER_CALIBRA_YAW,&ctrl.twist.angular.z);  
                            
//                            speed_clock_basket_calculate(&ctrl.twist.angular.z);
                            ctrl.chassis_ctrl = CHASSIS_LOW_MODE;   
                        }
                        
                        
                        
                            
                        
                    } 
                    #if CHANGE_MODE
                        ctrl.catch_ball = CATCH_OFF;
                        ctrl.car_comm_ctrl = CAR_COMMUICA_OFF;
                    #else
                    else if(_tool_Abs(air_joy.SWA - 2000) < 50) //SWA DOWN
                    {
                        if(_tool_Abs(air_joy.SWD - 1000) < 50 && _tool_Abs(air_joy.SWC - 1000) < 50)
                            ctrl.chassis_ctrl = CHASSIS_COM_MODE;   //普通移动
                        
                        else if(_tool_Abs(air_joy.SWD - 2000) < 50 && _tool_Abs(air_joy.SWC - 1000) < 50)
                            ctrl.chassis_ctrl = CHASSIS_LOW_MODE;   
                        
                    }
                    #endif
                }
                /*-========================================================-*/

                else if(_tool_Abs(air_joy.SWB - 2000) < 50)
                {
                    ctrl.pitch_ctrl = PITCH_AUTO_MODE;          //俯仰自动
                    ctrl.robot_crtl = ROAD_AUTO_MODE;   //自动模式
                    if(_tool_Abs(air_joy.SWA - 2000) < 50)
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOCK_TARGET;    //底盘锁定篮筐
   
                    }
                    else if(_tool_Abs(air_joy.SWA - 1000) < 50)
                    {
                        ctrl.chassis_ctrl = CHASSIS_LOW_MODE;       //底盘普通移动
                    }
                    
                } 

            
		    }
            else//所有机构全部关闭
            {
                ctrl.robot_crtl = OFF_MODE;
                ctrl.chassis_ctrl = CHASSIS_OFF;
                ctrl.pitch_ctrl = PITCH_RESET_MODE;
                ctrl.friction_ctrl = FRICTION_OFF_MODE;
                ctrl.shoot_ctrl = SHOOT_OFF;
                ctrl.catch_ball = CATCH_OFF;
                ctrl.car_comm_ctrl = CAR_COMMUICA_OFF;
                ctrl.laser_ctrl = LASER_CALIBRA_OFF;
                ctrl.dribble_ctrl = DRIBBLE_OFF;
                
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
