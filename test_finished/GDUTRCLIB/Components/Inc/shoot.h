/**
 * @file shoot.h
 * @author Wu Jia 
 * @brief Ͷ����Ϲ���ʵ��
 *        �����Ӧ��ʾ����ע������ȥshoot.cpp��
 */

#pragma once
#include <cmath>
#include <math.h>
#include <stdint.h>
#include <stddef.h>

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
        float a, b, c, d;  // ����ʽϵ����y = a + b*(x-x0) + c*(x-x0)^2 + d*(x-x0)^3
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

    /**
     * @brief 8.1 �����ݵ�
     */
    float GetShootSpeed_ByOne(float distance, const SplineSegment* segments);

    /**
     * @brief 8.7 ��ʱ���ݵ�
     */
    float GetShootSpeed_Beyond(float distance);


    /**
     * @brief  8.7 �����������ݵ�
     * 
     */
    float GetShootSpeed_OnSite(float distance);

    /**
     * @brief 8.8 ����
     */
    float GetShootSpeed_After(float distance);
    

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


/*�����Ǿɰ汾��*/

// #pragma once
// #include <cmath>
// #include <stdint.h>
// #include <vector>
// #include <iostream>
// #include <cassert>

// #define Simple_or_Not 0 //1Ϊʹ�ü�, 0Ϊʹ�����ԽǾ���


// /**
//  * @brief Ͷ������ϵͳ������-ת��ӳ�䣩
//  * @note  ʹ������������ֵ����ϲ�������
//  *        ֧�ִ�/С���ָ����Ƕ�ģʽ
//  */
// class ShootController{
// public:
    
//     struct SamplePoint 
//     {
//         float distance; // ���루�ף�
//         float speed;     // ��ӦĦ����eRPM
//     };

//     struct SplineParams     //�������������ṹ��(��)
//     {
//         float a, b, c, d;  //a + b * x + c * x^2 + d * x^3
//     };
//     /**
//      * @brief ��ʼ����������
//      * @param samples ����������ָ��
//      * @param count   ����������
//      * @param isLargePitch true=������ģʽ false=С����ģʽ
//      * @warning            ��������밴distance��������
//      */
//     void Init(const SamplePoint* samples, int count, bool isLargePitch);

//     /**
//      * @brief ����Ŀ��ת��
//      * @param distance    ��ǰ���루�ף�
//      * @param isLargePitch ����ģʽ
//      * @return Ŀ��ת�٣�eRPM�� 
//      * @note ����������Χʱ�Զ��ضϵ��߽�ֵ
//      * @attention ��ΪeRPM��������ʵRPM��RPM = eRPM / ������
//      */
//     float CalculateSpeed(float distance, bool isLargePitch) const;

// private:
//     // ��������ָ��
//     std::vector<SamplePoint> largePitchSamples;
//     std::vector<SamplePoint> smallPitchSamples;
//     int largePitchCount = 0;
//     int smallPitchCount = 0;
    
//     // ���������洢��ÿ����������֮��һ�������Σ�
//     std::vector<SplineParams> largePitchSplines; // ��������������
//     std::vector<SplineParams> smallPitchSplines; // С������������

//     /**
//      * @brief ������������������
//      * @param samples ����������
//      * @param count   ��������
//      * @param splines ��������洢
//      * @note ��ʵ��Ϊ�򻯰棬���������ʹ�����������ԽǾ����㷨
//      */
//     void BuildSpline_Simple(const std::vector<SamplePoint>& samples, int count, std::vector<SplineParams>& splines);

//     /**
//      * @brief ���ݲ�������������������ߵĲ���
//      * @param samples ����������
//      * @param count �����������
//      * @param splines ���������������������
//      * @note ʹ�õ������������ԽǾ����㷨
//      */

//     void BuildSpline(const std::vector<SamplePoint>& samples, std::vector<SplineParams>& splines);

//     /**
//      * @brief ������������
//      * @param distance �������
//      * @param samples  ����������
//      * @param splines  ��������
//      * @param count    ��������
//      * @return ��ֵ������
//      * @note ʹ�ö��ֲ��Ҷ�λ����
//      */
//     float EvaluateSpline(float distance, const std::vector<SamplePoint>& samples, 
//                          const std::vector<SplineParams>& splines, int count) const;
// };
