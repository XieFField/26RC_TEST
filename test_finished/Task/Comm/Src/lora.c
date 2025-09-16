/**
 * @file lora.c
 * @author Zhong Yi
 * @brief ˫��ͨѶģ��
 */

#include "lora.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include "drive_atk_mw1278d.h"
#include "FreeRTOS.h"
#include "task.h"

#include "position.h"

//#define LORA_ON 1

//#if ON

///* ATK-MW1278Dģ�����ò������� */
//#define DEMO_ADDR       0                               /* �豸��ַ */
//#define DEMO_WLRATE     ATK_MW1278D_WLRATE_19K2        /* �������� */
//#define DEMO_CHANNEL    0                               /* �ŵ� */
//#define DEMO_TPOWER     ATK_MW1278D_TPOWER_20DBM       /* ���书�� */
//#define DEMO_WORKMODE   ATK_MW1278D_WORKMODE_NORMAL    /* ����ģʽ */
//#define DEMO_TMODE      ATK_MW1278D_TMODE_TT           /* ����ģʽ */
//#define DEMO_WLTIME     ATK_MW1278D_WLTIME_1S          /* ����ʱ�� */
//#define DEMO_UARTRATE   ATK_MW1278D_UARTRATE_115200BPS /* UARTͨѶ������ */
//#define DEMO_UARTPARI   ATK_MW1278D_UARTPARI_NONE      /* UARTͨѶУ��λ */



//uint8_t times_error = 1;

//// ȫ�����ݻ�����(����ջռ��)
//static struct {
//    float num1[3];
//    float num2[3];
//    int   num3[3];
//} g_data_buf = {0};

//UBaseType_t stack_high_water_mark;

//// ȫ��״̬����
//static uint8_t g_idx = 0;            // ���λ���������(1�ֽ�)
//static uint8_t g_count = 0;          // ��Ч���ݼ���(1�ֽ�)
//static float   g_num1, g_num2;       // ��ʱ�������
//static int     g_num3;
//static char    g_parse_buf[32];      // �����û�����
//static uint8_t *g_buf;
//static uint16_t g_len;
//static int      g_valid;

//// ʾ�����ݺͷ��ͻ�����(�ϲ���������ڴ���Ƭ)
//    int temp=25545;
//    float flt1=581.575747f;
//    float flt2=3.1415926f;

//// ������UART���ͺ���(���printf)
//void uart_putc(char c) {
//    while((USART1->SR & 0X40) == 0);  // �ȴ��������
//    USART1->DR = (uint8_t)c;
//}

//// �Ż��ĸ�����ת�ַ�������(����ջʹ��)
//static uint8_t float_to_str(char *buf, float value, uint8_t decimals) {
//    int32_t integer_part = (int32_t)value;
//    int32_t fractional_part = (int32_t)((value - integer_part) * 
//                            (decimals == 1 ? 10 : decimals == 2 ? 100 : 1000));
//                            
//    uint8_t len = 0;
//    char temp[16];
//    int32_t num = integer_part;
//    int8_t i = 0;
//    
//    // ������
//    if (num < 0) {
//        *buf++ = '-';
//        len++;
//        num = -num;
//    }
//    
//    // ������������
//    if (num == 0) {
//        temp[i++] = '0';
//    } else {
//        while (num > 0) {
//            temp[i++] = (num % 10) + '0';
//            num /= 10;
//        }
//    }
//    
//    // ��ת��������
//    while (i > 0) {
//        *buf++ = temp[--i];
//        len++;
//    }
//    
//    // ����С������
//    if (decimals > 0) {
//        *buf++ = '.';
//        len++;
//        
//        // ȷ��С���������㹻��λ��
//        if (decimals == 2) {
//            *buf++ = (fractional_part / 10) % 10 + '0';
//            *buf++ = fractional_part % 10 + '0';
//            len += 2;
//        } else if (decimals == 1) {
//            *buf++ = fractional_part % 10 + '0';
//            len++;
//        }
//    }
//    
//    return len;
//}

//// �Ż��Ľ�������(�������ٵ��ÿ���)
//// �����Ľ�������
//static inline void parse_data(uint8_t *buf, uint16_t len) {
//    char *p = (char*)buf;
//    char *start = p;
//    
//    // ���Ƹ��Ƴ��ȣ��������
//    len = (len > 31) ? 31 : len;
//    memcpy(g_parse_buf, p, len);
//    g_parse_buf[len] = '\0';
//    
//    // ����num1
//    g_num1 = atof(start);
//    
//    // ���ҵ�һ������
//    p = strchr(start, ',');
//    if (!p) {
//        // û���ҵ����ţ�����ʧ��
//        g_num2 = 0;
//        g_num3 = 0;
//        return;
//    }
//    
//    // ����num2
//    p++; // ��������
//    g_num2 = atof(p);
//    
//    // ���ҵڶ�������
//    p = strchr(p, ',');
//    if (!p) {
//        // û���ҵ��ڶ������ţ�����ʧ��
//        g_num3 = 0;
//        return;
//    }
//    
//    // ����num3
//    p++; // ��������
//    g_num3 = atoi(p);
//}

//// ���Ŷ��ж�(�������ٵ��ÿ���)
//static inline int is_valid_data(void) {
//    // ֻ������µ��������ݵ㣬���ټ�����
//    uint8_t prev_idx = (g_idx + 2) % 3;  // ǰһ������
//    
//    // ���num1
//    float diff_f = (g_data_buf.num1[g_idx] > g_data_buf.num1[prev_idx]) ?
//                  (g_data_buf.num1[g_idx] - g_data_buf.num1[prev_idx]) :
//                  (g_data_buf.num1[prev_idx] - g_data_buf.num1[g_idx]);
//    if(diff_f >= 2) return 0;
//    
//    // ���num2
//    diff_f = (g_data_buf.num2[g_idx] > g_data_buf.num2[prev_idx]) ?
//            (g_data_buf.num2[g_idx] - g_data_buf.num2[prev_idx]) :
//            (g_data_buf.num2[prev_idx] - g_data_buf.num2[g_idx]);
//    if(diff_f >= 2) return 0;
//    
//    // ���num3
//    int diff_i = (g_data_buf.num3[g_idx] > g_data_buf.num3[prev_idx]) ?
//                (g_data_buf.num3[g_idx] - g_data_buf.num3[prev_idx]) :
//                (g_data_buf.num3[prev_idx] - g_data_buf.num3[g_idx]);
//    if(diff_i >= 2) return 0;
//    
//    return 1;
//}

//// ģ���ʼ������
//void All_Init(void) {
//    uint8_t ret;
//    
//    // ��ʼ��ATK-MW1278Dģ��
//    ret = atk_mw1278d_init(115200);
//    if(ret != 0) while(1) osDelay(2);
//    
//    // ��������ģʽ
//    atk_mw1278d_enter_config();
//    
//    // ���ò���
//    ret  = atk_mw1278d_addr_config(DEMO_ADDR);
//    ret += atk_mw1278d_wlrate_channel_config(DEMO_WLRATE, DEMO_CHANNEL);
//    ret += atk_mw1278d_tpower_config(DEMO_TPOWER);
//    ret += atk_mw1278d_workmode_config(DEMO_WORKMODE);
//    ret += atk_mw1278d_tmode_config(DEMO_TMODE);
//    ret += atk_mw1278d_wltime_config(DEMO_WLTIME);
//    ret += atk_mw1278d_uart_config(DEMO_UARTRATE, DEMO_UARTPARI);
//    
//    // �˳�����ģʽ
//    atk_mw1278d_exit_config();
//    
//    if(ret != 0) while(1) osDelay(2);
//    
//    // ��������
//    atk_mw1278d_uart_rx_restart();
//}

//#endif
int clock=1;
/**
 * @brief       LORA��������(�Ż���)
 * @param       argument: �������
 * @retval      ��
 */
void Lora_Task(void *argument) 
{
	for(;;) 
    {
	      osDelay(5);

    }
        
        
		
	
}

/*���ڲ���*/

extern int32_t speed1;
extern int32_t speed2;
extern int32_t speed3;

/**
 * @brief       LORA��������(�Ż���)
 * @param       argument: �������
 * @retval      ��
 */
float temp_x;
float temp_y;
void Lora_Task1(void *argument) 
{
    uint8_t len1, len2;
    char *p;

    for(;;) 
    {      
//    temp_x=RealPosData.world_x;
//	temp_y=RealPosData.world_y;
//    atk_mw1278d_uart_printf("%d,%d,%d", (int)((temp_x - 3.6690f)*1000 ) , (int)((temp_y + 0.6217f)*1000 ), 123);

    //printf_UART("%d,%d,%d\n",speed1,speed2,speed3);
        osDelay(300);
 
        /*��ʱ������vofaͨ��*/
        
      
	}
}

void POS_Send(float x, float y ,int z)
{
    x = y;
}

void clock_change(int c)
{
    clock = c;
}
