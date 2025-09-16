/**
 * @file LED.cpp
 * @author Wu Jia
 * @brief ָʾ��
 */

#include "LED.h" 
#include "drive_tim.h"


static LED_MODE_T current_mode = LED_MODE_NORMAL;  

static const TickType_t FLASH_ON_MS = pdMS_TO_TICKS(700);    
static const TickType_t FLASH_PERIOD_MS = pdMS_TO_TICKS(1400);  

Ws2812b_SIGNAL_T signal;

void LED_Task(void *pvParameters)
{
    for(;;)
    {
        if(xQueueReceive(LED_Port, &signal, 0) == pdPASS)
        {
            switch(signal)
            {
                case SIGNAL_NORMAL:
                    current_mode = LED_MODE_NORMAL;  // ������ģʽ(��ɫ)
                    break;
                case SIGNAL_CATCH:
                    current_mode = LED_MODE_CATCH;   // ����(��ɫ)
                    break;
                case SIGNAL_FAIL:
                    current_mode = LED_MODE_FAIL;    // ��λ���ϣ���˸��
                    break;
                case SIGNAL_WAIT:
                    current_mode = LED_MODE_WAIT;    // �ض�λ����˸��
                    break;
				case SIGNAL_SHOOT:
					current_mode = LED_MODE_SHOOT;	// ��ս��ģʽ(��ɫ)
					break;
                default:
                    current_mode = LED_MODE_OFF;     // Ĭ�����
                    break;
            }
        }

        switch(current_mode)
        {
            case LED_MODE_NORMAL:
                LED_NORMAL();  //��
                WS2812b_Send();
                break;

            case LED_MODE_CATCH:
                LED_CATCH();   //��
                WS2812b_Send();
                break;
			case LED_MODE_SHOOT:
				LED_SHOOT();
				WS2812b_Send();
				break;

            case LED_MODE_FAIL:
            case LED_MODE_WAIT:

                TickType_t current_tick = xTaskGetTickCount();

                TickType_t tick_in_period = current_tick % FLASH_PERIOD_MS;

                if(tick_in_period < FLASH_ON_MS)
                {

                    if(current_mode == LED_MODE_FAIL)
                        LED_FAIL();
                    else
                        LED_WAIT();
                }
                else
                {

                    LED_OFF();
                }
                WS2812b_Send();  
                break;

            case LED_MODE_OFF:
                LED_OFF();       // ���
                WS2812b_Send();
                break;
        }


        osDelay(10);
    }
}