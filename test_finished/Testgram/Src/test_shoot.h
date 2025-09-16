
#pragma once
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <iostream>

#define PI 3.14159265358979f

class ShootController {
public:

    ShootController();
    ~ShootController() = default;

    /**
     * @brief �����β����ṹ�壨ÿ��һ����
     * 
     * @attention �ڼ���������ֵʱ��ÿһ�������������㶼��Ӧ��һ�������Σ�Ҳ��������������У�
     *            ÿ�������ڵĵ�֮����һ����Ӧ�� SplineSegment����ˣ�������� num �������㣬
     *            ����ʵ����ֻ�� num-1 ��������
     */
    typedef struct {
        float a, b, c, d;  // ����ʽϵ����y = d + c*(x-x0) + b*(x-x0)^2 + a*(x-x0)^3
    } SplineSegment;

    typedef struct{
        float hoop_distance;
        float hoop_angle;
        float shoot_speed;
    } Shoot_Info_E;

     /**
     * @brief ��ʼ����������
     * @param segments ��������
     * @param sample_distance ������������
     * @param num ������
     * @param whichPitch �Ƿ������
     */
    void Init(const SplineSegment* segments, const float* sample_distance, uint16_t num, int whichPitch);

     /**
     * @brief ��ȡĿ��ת��
     * @param distance ��ǰ����
     * @param isLargePitch �Ƿ������
     * @return Ŀ��ת��
     */
    float GetShootSpeed(float distance, int whichPitch);

    /**
     * @brief ���ݵ��������������������ˮƽ���룬�Լ��н�
     * @param hoop_x  ����x����   ��λ:M
     * @param hoop_y  ����y����   ��λ:M
     * @param robot_x ������x���� ��λ:M
     * @param robot_y ������y���� ��λ:M
     * @param info �������ݽṹ��
     */
    void GetShootInfo(float hoop_x, float hoop_y, float robot_x, float robot_y, Shoot_Info_E *info);

    
private:
    const SplineSegment* largePitchTable = nullptr;
    size_t largePitchCount = 0;

    const SplineSegment* smallPitchTable = nullptr;
    size_t smallPitchCount = 0;

    const SplineSegment* midPitchTable = nullptr;
    size_t midPitchCount = 0;

    const float* largePitchDistances = nullptr;
    const float* smallPitchDistances = nullptr;
    const float* midPitchDistances = nullptr;

    int lastWhichPitch = 0;
    int lastspeed = 0;

    const SplineSegment* getSplineTable(int whichPitch) const;
    const float* getDistanceTable(int whichPitch) const;
    size_t getCount(int whichPitch) const;

    float CalcSpeed(float distance, const SplineSegment* cubic_spline, const float* sample_distance, uint16_t num);
    int FindSegment(float distance, const float* sample_distance, uint16_t num) const;
};