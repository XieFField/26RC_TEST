#include "DEMO.h"

namespace geometry {

// 构造函数
Point2D::Point2D(float x, float y) : x(x), y(y) {}
Point3D::Point3D(float x, float y, float z) : x(x), y(y), z(z) {}

/* ---------- 2D 齐次变换 ---------- */
// 默认构造函数，初始化为单位矩阵
HomogeneousTransform<false>::HomogeneousTransform() { identity(); }

// 带参数的构造函数，设置平移和旋转
HomogeneousTransform<false>::HomogeneousTransform(float tx, float ty, float theta) {
    setTransform(tx, ty, theta);
}

// 设置为单位矩阵
void HomogeneousTransform<false>::identity() {
    for (int i = 0; i < 9; ++i) m[i] = 0.0f;
    m[0] = m[4] = m[8] = 1.0f; // 对角线元素设为1
}

// 设置完整的2D变换矩阵（平移+旋转）
void HomogeneousTransform<false>::setTransform(float tx, float ty, float theta) {
    float c = cosf(theta), s = sinf(theta);
    m[0] = c;  m[1] = -s; m[2] = tx; // 旋转和平移部分
    m[3] = s;  m[4] = c;  m[5] = ty;
    m[6] = 0;  m[7] = 0;  m[8] = 1;  // 齐次坐标部分
}

// 仅设置平移部分
void HomogeneousTransform<false>::setTranslation(float x, float y) {
    m[2] = x; m[5] = y;
}

// 仅设置旋转部分（保持原有平移不变）
void HomogeneousTransform<false>::setRotation(float theta) {
    float tx = m[2], ty = m[5]; // 保存当前平移
    float c = cosf(theta), s = sinf(theta);
    m[0] = c; m[1] = -s; m[2] = tx; // 设置旋转并恢复平移
    m[3] = s; m[4] = c;  m[5] = ty;
}

// 应用变换到坐标点（输出到参数）
void HomogeneousTransform<false>::apply(float x, float y, float& outX, float& outY) const {
    outX = m[0]*x + m[1]*y + m[2];
    outY = m[3]*x + m[4]*y + m[5];
}

// 应用变换到Point2D对象（返回新点）
Point2D HomogeneousTransform<false>::apply(const Point2D& p) const {
    return Point2D(m[0]*p.x + m[1]*p.y + m[2],
                   m[3]*p.x + m[4]*p.y + m[5]);
}

// 矩阵乘法：当前变换与另一个变换相乘
HomogeneousTransform<false>
HomogeneousTransform<false>::multiply(const HomogeneousTransform& o) const {
    HomogeneousTransform r;
    // 3x3矩阵乘法
    r.m[0] = m[0]*o.m[0] + m[1]*o.m[3] + m[2]*o.m[6];
    r.m[1] = m[0]*o.m[1] + m[1]*o.m[4] + m[2]*o.m[7];
    r.m[2] = m[0]*o.m[2] + m[1]*o.m[5] + m[2]*o.m[8];

    r.m[3] = m[3]*o.m[0] + m[4]*o.m[3] + m[5]*o.m[6];
    r.m[4] = m[3]*o.m[1] + m[4]*o.m[4] + m[5]*o.m[7];
    r.m[5] = m[3]*o.m[2] + m[4]*o.m[5] + m[5]*o.m[8];

    r.m[6] = m[6]*o.m[0] + m[7]*o.m[3] + m[8]*o.m[6];
    r.m[7] = m[6]*o.m[1] + m[7]*o.m[4] + m[8]*o.m[7];
    r.m[8] = m[6]*o.m[2] + m[7]*o.m[5] + m[8]*o.m[8];
    return r;
}

// 计算变换矩阵的逆
HomogeneousTransform<false>
HomogeneousTransform<false>::inverse() const {
    HomogeneousTransform r;
    // 计算行列式
    float det = m[0]*(m[4]*m[8] - m[5]*m[7])
              - m[1]*(m[3]*m[8] - m[5]*m[6])
              + m[2]*(m[3]*m[7] - m[4]*m[6]);
    if (det == 0.0f) { r.identity(); return r; } // 行列式为0，返回单位矩阵
    float invD = 1.0f / det;
    // 计算伴随矩阵并除以行列式
    r.m[0] = (m[4]*m[8] - m[5]*m[7]) * invD;
    r.m[1] = (m[2]*m[7] - m[1]*m[8]) * invD;
    r.m[2] = (m[1]*m[5] - m[2]*m[4]) * invD;
    r.m[3] = (m[5]*m[6] - m[3]*m[8]) * invD;
    r.m[4] = (m[0]*m[8] - m[2]*m[6]) * invD;
    r.m[5] = (m[2]*m[3] - m[0]*m[5]) * invD;
    r.m[6] = (m[3]*m[7] - m[4]*m[6]) * invD;
    r.m[7] = (m[1]*m[6] - m[0]*m[7]) * invD;
    r.m[8] = (m[0]*m[4] - m[1]*m[3]) * invD;
    return r;
}

/* ---------- 3D 齐次变换 ---------- */
// 默认构造函数，初始化为单位矩阵
HomogeneousTransform<true>::HomogeneousTransform() { identity(); }

// 带参数的构造函数，设置平移和欧拉角旋转
HomogeneousTransform<true>::HomogeneousTransform(float tx, float ty, float tz,
                                                 float roll, float pitch, float yaw)
{ setTransform(tx, ty, tz, roll, pitch, yaw); }

// 设置为单位矩阵
void HomogeneousTransform<true>::identity() {
    for (int i = 0; i < 16; ++i) data[i] = 0.0f;
    data[0] = data[5] = data[10] = data[15] = 1.0f; // 对角线元素设为1
}

// 设置完整的3D变换矩阵（平移+欧拉角旋转）
void HomogeneousTransform<true>::setTransform(float tx, float ty, float tz,
                                              float roll, float pitch, float yaw) {
    setTranslation(tx, ty, tz);
    setRotation(roll, pitch, yaw);
}

// 仅设置平移部分
void HomogeneousTransform<true>::setTranslation(float x, float y, float z) {
    data[3] = x; data[7] = y; data[11] = z;
}

// 仅设置旋转部分（使用欧拉角：roll, pitch, yaw）
void HomogeneousTransform<true>::setRotation(float roll, float pitch, float yaw) {
    float sr = sinf(roll),  cr = cosf(roll);
    float sp = sinf(pitch), cp = cosf(pitch);
    float sy = sinf(yaw),   cy = cosf(yaw);
    float tx = data[3], ty = data[7], tz = data[11]; // 保存当前平移

    // 构建旋转矩阵（Z-Y-X顺序）
    data[0] = cy * cp;
    data[1] = cy * sp * sr - sy * cr;
    data[2] = cy * sp * cr + sy * sr;
    data[3] = tx; // 恢复平移

    data[4] = sy * cp;
    data[5] = sy * sp * sr + cy * cr;
    data[6] = sy * sp * cr - cy * sr;
    data[7] = ty; // 恢复平移

    data[8]  = -sp;
    data[9]  = cp * sr;
    data[10] = cp * cr;
    data[11] = tz; // 恢复平移
}

// 应用变换到坐标点（输出到参数）
void HomogeneousTransform<true>::apply(float x, float y, float z,
                                       float& outX, float& outY, float& outZ) const {
    outX = data[0]*x + data[1]*y + data[2]*z + data[3];
    outY = data[4]*x + data[5]*y + data[6]*z + data[7];
    outZ = data[8]*x + data[9]*y + data[10]*z + data[11];
}

// 应用变换到Point3D对象（返回新点）
Point3D HomogeneousTransform<true>::apply(const Point3D& p) const {
    return Point3D(
        data[0]*p.x + data[1]*p.y + data[2]*p.z + data[3],
        data[4]*p.x + data[5]*p.y + data[6]*p.z + data[7],
        data[8]*p.x + data[9]*p.y + data[10]*p.z + data[11]);
}

// 矩阵乘法：当前变换与另一个变换相乘
HomogeneousTransform<true>
HomogeneousTransform<true>::multiply(const HomogeneousTransform& o) const {
    HomogeneousTransform r;
    // 4x4矩阵乘法
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += data[i*4+k] * o.data[k*4+j];
            r.data[i*4+j] = s;
        }
    return r;
}

// 计算变换矩阵的逆（对于刚体变换的简化计算）
HomogeneousTransform<true>
HomogeneousTransform<true>::inverse() const {
    HomogeneousTransform r;
    // 旋转矩阵的逆等于其转置
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.data[i*4+j] = data[j*4+i];
    // 计算平移部分的逆：-R^T * t
    r.data[3]  = -(r.data[0]*data[3] + r.data[1]*data[7] + r.data[2]*data[11]);
    r.data[7]  = -(r.data[4]*data[3] + r.data[5]*data[7] + r.data[6]*data[11]);
    r.data[11] = -(r.data[8]*data[3] + r.data[9]*data[7] + r.data[10]*data[11]);
    r.data[15] = 1.0f; // 齐次坐标部分
    return r;
}

// 运行示例演示函数
DemoResult runDemo()
{
    using namespace geometry;
    ReceiveRealData_S robotPos; // 声明但未初始化的位置变量
    
    // 初始化结果结构体
    DemoResult r = {0};
    
    // 1. 2D变换演示
    HomogeneousTransform<false> T1;
    T1.setTransform(1.0f, 1.0f, M_PI); // 设置沿x轴平移1，沿y轴平移1，旋转180度
    Point2D p2d(robotPos.x, robotPos.y); // 使用未初始化的值，实际应用中应从队列获取
    r.q2d = T1.apply(p2d); // 应用变换

    // 2. 3D变换演示：绕Z轴旋转90度，沿X轴平移1
    HomogeneousTransform<true> T2;
    T2.setTransform(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, M_PI/2);
    Point3D p3d(1.0f, 0.0f, 0.0f); // 点(1,0,0)
    r.q3d = T2.apply(p3d); // 应用变换
    
    return r;
}

} // namespace geometry