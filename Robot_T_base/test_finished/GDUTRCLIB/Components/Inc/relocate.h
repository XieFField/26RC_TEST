#ifndef RELOCATE_H
#define RELOCATE_H

#include "position.h"
#include "LaserPositioning_Task.h"

//����
struct  position2D
{
    /* data */
    float x;
    float y;


//    // ���ؼӷ�����������˲�����
//    position2D operator+(const position2D& other) const 
//    {
//        return position2D(x + other.x, y + other.y);
//    }

//    // �����˷�
//    position2D operator*(float scalar) const 
//    {
//        return position2D(x * scalar, y * scalar);
//    }
};

// ���ر߽����
struct FieldBoundary 
{
    float x_min;  // ��߽�X����
    float x_max;  // �ұ߽�X����
    float y_min;  // �±߽�Y����
    float y_max;  // �ϱ߽�Y����
};

// �������ֵ
struct LaserMeasurement 
{
    float d_back;  // ���򼤹����ֵ ��Ӧy�Ḻ����
    float d_left; //  ���򼤹����ֵ ��Ӧx�Ḻ����
};

// �ֶ�ʵ�ֵ�min/max����
inline float min_float(float a, float b) 
{
    return a < b ? a : b;
}

inline float max_float(float a, float b) 
{
    return a > b ? a : b;
}


#ifdef __cplusplus
extern "C" {
#endif 



class Relocation{
public:

    void init(const position2D&initPos)
    {
        estimatedPos = initPos;
    }


    Relocation(float alpha_input, FieldBoundary boundary_input) 
    {
        this->alpha = alpha_input;
        boundary = boundary_input;
    }
      


    position2D LaserPosition_calc(const LaserMeasurement& meas); 


    position2D updatePositionData(const position2D& laserPos, 
        const position2D& odomDelta);
    

private:

    const position2D get_LaserData();

    position2D estimatedPos; //����λ��

    FieldBoundary boundary;
    float alpha;           // �˲�ϵ�� 
};


class Reposition{
public:
    Reposition(float x, float y)
    {
        hoop_X = x;
        hoop_Y = y;
    }

    position2D Reposition_Calc_VPC(float distance, float yaw);

private:
    float hoop_X = 0;
    float hoop_Y = 0;
};

#ifdef __cplusplus
}
#endif

#endif
