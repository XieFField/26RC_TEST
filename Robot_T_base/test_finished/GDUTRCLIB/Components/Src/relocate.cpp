/**
 * @file relocate.h
 * @author Wu Jia
 * @version 0.1
 * @brief �ض�λ����Ҫ�ǻ����˲�
 *        ��ʵ�����˲��������ɹ����࣬��������26rcʱ����������
 *        
 *        ��Ȼ��ս���������䳡�ز�ͬ������������ϵ������ͬһ����
 *        ֻҪ�任���������Ϳ���
 *        ���Ŀǰֻ���ڼ����ض�λ�����������Ӿ���λ����Ļ������Խ�Ͻ���
 * 
 *        �����������ͼ����Ϊ������򣬲����Կ������ض�λ
 * 
 * 
 * @brief ��̼ƾ���Ʈ�����������Բ����˲�������ȷ����������ֱ�����ض�λ���ݸ���
 */

#include "relocate.h"

position2D Relocation::LaserPosition_calc(const LaserMeasurement& meas)
{
    position2D result;

    result.x = boundary.x_min + meas.d_left;

    result.y = boundary.y_min + meas.d_back;

    // �߽����ƣ�ȷ�������ڳ��ط�Χ�ڣ�0 �� x �� x_max��0 �� y �� y_max��
    result.x = max_float(0.0f, min_float(result.x, boundary.x_max));

    result.y = max_float(0.0f, min_float(result.y, boundary.y_max));

    return result;
}

position2D Relocation::updatePositionData(const position2D& laserPos, 
        const position2D& odomDelta)
{
    position2D odom_pos;

    odom_pos.x = estimatedPos.x + odomDelta.x;
    odom_pos.y = estimatedPos.y + odomDelta.y;

    estimatedPos.x = (odom_pos.x * (1.0f - alpha) + (laserPos.x * alpha));
    estimatedPos.y = (odom_pos.y * (1.0f - alpha) + (laserPos.y * alpha));

    return odom_pos;
}

position2D Reposition::Reposition_Calc_VPC(float distance_, float yaw)
{
    float distance = distance_ + 15.5;
    position2D temp_value;
    float yaw_rad = yaw * PI / 180.0;
    if(yaw > 0)
    {
        temp_value.y = hoop_Y - distance * cos(yaw_rad);
        temp_value.x = hoop_X - distance * sin(yaw_rad);
    }
    else if(yaw < 0)
    {
        temp_value.y = hoop_Y - distance * cos(yaw_rad);
        temp_value.x = hoop_X + distance * sin(yaw_rad);
    }
    else
    {
        temp_value.x = hoop_X;
        temp_value.y = hoop_Y - distance;
    }
    temp_value.x = -temp_value.x;
    return temp_value;
}

