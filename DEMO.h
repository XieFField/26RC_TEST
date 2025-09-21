#ifndef DEMO_H
#define DEMO_H

// 数学常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 必要的头文件包含
#include <cmath> // 数学函数
#include <math.h>
#include "cmsis_os.h"            // FreeRTOS 内核
#include "FreeRTOS.h"            // FreeRTOS 头文件
#include "task.h"                // FreeRTOS 任务管理
#include "drive_uart.h"          // 串口驱动
#include "ViewCommunication.h"   // 视觉通信
#include "queue.h"               // FreeRTOS 队列
#include "arm_math.h"
namespace geometry {

// 2D点结构体
struct Point2D {
    float x, y;
    Point2D(float x = 0.0f, float y = 0.0f); // 构造函数
};

// 3D点结构体
struct Point3D {
    float x, y, z;
    Point3D(float x = 0.0f, float y = 0.0f, float z = 0.0f); // 构造函数
};

// 演示结果结构体
struct DemoResult {
    geometry::Point2D q2d;      // 2D 变换结果
    geometry::Point3D q3d;      // 3D 变换结果
    float             x, y;     // 逆变换后再投影的 2D 点
};

// 前置声明
template <bool is3D>
class HomogeneousTransform;

// 2D 齐次变换特化
template <>
class HomogeneousTransform<false> {
public:
    HomogeneousTransform(); // 默认构造函数
    HomogeneousTransform(float tx, float ty, float theta); // 带参数构造函数

    void identity(); // 设置为单位矩阵
    void setTransform(float tx, float ty, float theta); // 设置完整变换
    void setTranslation(float x, float y); // 仅设置平移
    void setRotation(float theta); // 仅设置旋转

    // 应用变换到坐标
    void apply(float x, float y, float& outX, float& outY) const;
    Point2D apply(const Point2D& point) const; // 应用变换到点

    HomogeneousTransform multiply(const HomogeneousTransform& other) const; // 矩阵乘法
    HomogeneousTransform inverse() const; // 矩阵求逆

private:
    float m[9]; // 3x3 行主序矩阵
};

// 3D 齐次变换特化
template <>
class HomogeneousTransform<true> {
public:
    HomogeneousTransform(); // 默认构造函数
    HomogeneousTransform(float tx, float ty, float tz,
                         float roll, float pitch, float yaw); // 带参数构造函数

    void identity(); // 设置为单位矩阵
    void setTransform(float tx, float ty, float tz,
                      float roll, float pitch, float yaw); // 设置完整变换
    void setTranslation(float x, float y, float z); // 仅设置平移
    void setRotation(float roll, float pitch, float yaw); // 仅设置旋转

    // 应用变换到坐标
    void apply(float x, float y, float z,
               float& outX, float& outY, float& outZ) const;
    Point3D apply(const Point3D& point) const; // 应用变换到点

    HomogeneousTransform multiply(const HomogeneousTransform& other) const; // 矩阵乘法
    HomogeneousTransform inverse() const; // 矩阵求逆

private:
    float data[16]; // 4x4 行主序矩阵
};

// 运行演示函数声明
DemoResult runDemo();
} // namespace geometry
#endif // DEMO_H