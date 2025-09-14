/**
 * @file test_shoot.cpp
 * @author Wu Jia
 * @brief ���Բ����
 */
#include "test_shoot.h"

ShootController::ShootController() {
    // ���գ����߳�ʼ����Ա
}

void ShootController::Init(const SplineSegment* segments, const float* sample_distance, uint16_t num, int whichLargePitch)
{
    /**
     * @brief �����Ƿ�����ǣ���ʼ����ͬ���������ݱ�;�������
     * @bug   Ŀǰ����û�жԲ������м��(SplineSegment* segments, float* sample_distance)
     *        ��bug�ѽ��
     */
    if (!segments || !sample_distance || num < 2)  // ������Ҫ��������������γ�һ����
        return;

    if (whichLargePitch == 3) 
    {
        largePitchTable = segments;
        largePitchDistances = sample_distance;
        largePitchCount = num;
    } 
    else if(whichLargePitch == 2)
    {
        midPitchTable = segments;
        midPitchDistances = sample_distance;
        midPitchCount = num;
    }
    else
    {
        smallPitchTable = segments;
        smallPitchDistances = sample_distance;
        smallPitchCount = num;
    }
}

int ShootController::FindSegment(float distance, const float* sample_distance, uint16_t num) const 
{
    /**
     * @brief ͨ�����ֲ����㷨���Ҿ��������ĸ�����������ĸ��Σ������ظö�����
     *      
     * @bug   ��distanceǡ�õ���sample_distance[num - 1]ʱ����ǰʵ�ֵ��߼����ܻ������
     *        ���ܻ���ҵ�right
     *        ��Ȼ��������ֵĸ��ʺ�С�����ǻ��ǸĽ�һ�±ȽϺ�
     *        �о������ܰ�while������Ϊleft <= right -1���ܽ����Ŀǰ��ȷ�����Ȼ�ͷ�Ի�����һ��
     */
    if (distance < sample_distance[0] || distance > sample_distance[num - 1]) 
    {
        if (distance < sample_distance[0])// ������Ч����
            return -2;  //С��
        if (distance > sample_distance[num - 1])
            return -3;  //����
    }

    int left = 0, right = num - 1, mid;  //���Ʋ����������� num - 2 ���������
    // while (left < right - 1) 
    // {
    //     int mid = (left + right) / 2;
    //     if (distance < sample_distance[mid]) 
    //         right = mid;

    //     else 
    //         left = mid;
    // }
    while (left < right)
	{
		mid = (left + right) / 2 + 1;
		
		if (distance <= sample_distance[mid])
		{
			right = mid - 1;
		}
		else
		{
			left = mid;
		}
	}

    return left;
}

float ShootController::CalcSpeed(float distance, const SplineSegment* cubic_spline, const float* sample_distance, uint16_t num) 
{
    /**
     * @brief ����Ŀǰ�����ľ��룬��������������ֵ���߼����Ŀ��ת�� 
     */
    int idx = FindSegment(distance, sample_distance, num);

    /*ȡ0��*/
    if (idx == -2)  //С��
        return 0.0f;
    if (idx == -3)  //����
        return 0.0f;


    // if (idx < -1)    �ɰ�
    // {
    //     if (distance < sample_distance[0])  // �������С����С�������룬���ص�һ���ε��ٶ�
    //         return cubic_spline[0].a;
    //     else                                // ��������������������룬�������һ���ε��ٶ�
    //         return cubic_spline[num - 2].a + cubic_spline[num - 2].b * (sample_distance[num - 1] - sample_distance[num - 2]);
    // }

    //�������뷶Χ����������

    float dx = distance - sample_distance[idx];
    // const SplineSegment& seg = cubic_spline[idx];
    const SplineSegment& seg = cubic_spline[idx];
    // return seg.d + seg.c * dx + seg.b * dx * dx + seg.a * dx * dx * dx;
    printf("������ţ�%d",idx);
    printf("%f, %f, %f, %f \n",cubic_spline[idx].a, cubic_spline[idx].b, cubic_spline[idx].c, cubic_spline[idx].d);
    return cubic_spline[idx].a * powf(dx,3) + cubic_spline[idx].b * powf(dx,2) + 
           cubic_spline[idx].c * dx + cubic_spline[idx].d;
    //return seg.a + seg.b * dx + seg.c * dx * dx + seg.d * dx * dx * dx;
}

float ShootController::GetShootSpeed(float distance, int whichPitch) 
{
    /**
     * @brief ���ݴ�С�����ǲþ����ĸ�����
     */

    if (whichPitch == 3) 
    {
        return CalcSpeed(distance, largePitchTable, largePitchDistances, largePitchCount);
    } 
    else if(whichPitch == 2)
    {
        return CalcSpeed(distance, midPitchTable, midPitchDistances, midPitchCount);
    }
    else 
    {
        return CalcSpeed(distance, smallPitchTable, smallPitchDistances, smallPitchCount);
    }
}

void ShootController::GetShootInfo(float hoop_x, float hoop_y, float robot_x, float robot_y, Shoot_Info_E *info)
{
    float dx, dy, target_Yaw;
    dx = hoop_x - robot_x;
    dy = hoop_y - robot_y;

    target_Yaw = atan2f(dy, dx);
    target_Yaw = target_Yaw * 180.0f / PI;

    if (target_Yaw <= -90.f)
	{
		target_Yaw += 270.f;
	}
	else
	{
		target_Yaw -= 90.f;
	}

    std::cout << "dx: " << dx << ", dy: " << dy << std::endl;
    std::cout << "raw distance: " << sqrtf(dx*dx + dy*dy) << std::endl;

    info->hoop_angle = target_Yaw;
    info->hoop_distance = sqrtf(dx * dx + dy * dy);   
}