#include "DEMO.h"

namespace geometry {

// ���캯��
Point2D::Point2D(float x, float y) : x(x), y(y) {}
Point3D::Point3D(float x, float y, float z) : x(x), y(y), z(z) {}

/* ---------- 2D ��α任 ---------- */
// Ĭ�Ϲ��캯������ʼ��Ϊ��λ����
HomogeneousTransform<false>::HomogeneousTransform() { identity(); }

// �������Ĺ��캯��������ƽ�ƺ���ת
HomogeneousTransform<false>::HomogeneousTransform(float tx, float ty, float theta) {
    setTransform(tx, ty, theta);
}

// ����Ϊ��λ����
void HomogeneousTransform<false>::identity() {
    for (int i = 0; i < 9; ++i) m[i] = 0.0f;
    m[0] = m[4] = m[8] = 1.0f; // �Խ���Ԫ����Ϊ1
}

// ����������2D�任����ƽ��+��ת��
void HomogeneousTransform<false>::setTransform(float tx, float ty, float theta) {
    float c = cosf(theta), s = sinf(theta);
    m[0] = c;  m[1] = -s; m[2] = tx; // ��ת��ƽ�Ʋ���
    m[3] = s;  m[4] = c;  m[5] = ty;
    m[6] = 0;  m[7] = 0;  m[8] = 1;  // ������겿��
}

// ������ƽ�Ʋ���
void HomogeneousTransform<false>::setTranslation(float x, float y) {
    m[2] = x; m[5] = y;
}

// ��������ת���֣�����ԭ��ƽ�Ʋ��䣩
void HomogeneousTransform<false>::setRotation(float theta) {
    float tx = m[2], ty = m[5]; // ���浱ǰƽ��
    float c = cosf(theta), s = sinf(theta);
    m[0] = c; m[1] = -s; m[2] = tx; // ������ת���ָ�ƽ��
    m[3] = s; m[4] = c;  m[5] = ty;
}

// Ӧ�ñ任������㣨�����������
void HomogeneousTransform<false>::apply(float x, float y, float& outX, float& outY) const {
    outX = m[0]*x + m[1]*y + m[2];
    outY = m[3]*x + m[4]*y + m[5];
}

// Ӧ�ñ任��Point2D���󣨷����µ㣩
Point2D HomogeneousTransform<false>::apply(const Point2D& p) const {
    return Point2D(m[0]*p.x + m[1]*p.y + m[2],
                   m[3]*p.x + m[4]*p.y + m[5]);
}

// ����˷�����ǰ�任����һ���任���
HomogeneousTransform<false>
HomogeneousTransform<false>::multiply(const HomogeneousTransform& o) const {
    HomogeneousTransform r;
    // 3x3����˷�
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

// ����任�������
HomogeneousTransform<false>
HomogeneousTransform<false>::inverse() const {
    HomogeneousTransform r;
    // ��������ʽ
    float det = m[0]*(m[4]*m[8] - m[5]*m[7])
              - m[1]*(m[3]*m[8] - m[5]*m[6])
              + m[2]*(m[3]*m[7] - m[4]*m[6]);
    if (det == 0.0f) { r.identity(); return r; } // ����ʽΪ0�����ص�λ����
    float invD = 1.0f / det;
    // ���������󲢳�������ʽ
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

/* ---------- 3D ��α任 ---------- */
// Ĭ�Ϲ��캯������ʼ��Ϊ��λ����
HomogeneousTransform<true>::HomogeneousTransform() { identity(); }

// �������Ĺ��캯��������ƽ�ƺ�ŷ������ת
HomogeneousTransform<true>::HomogeneousTransform(float tx, float ty, float tz,
                                                 float roll, float pitch, float yaw)
{ setTransform(tx, ty, tz, roll, pitch, yaw); }

// ����Ϊ��λ����
void HomogeneousTransform<true>::identity() {
    for (int i = 0; i < 16; ++i) data[i] = 0.0f;
    data[0] = data[5] = data[10] = data[15] = 1.0f; // �Խ���Ԫ����Ϊ1
}

// ����������3D�任����ƽ��+ŷ������ת��
void HomogeneousTransform<true>::setTransform(float tx, float ty, float tz,
                                              float roll, float pitch, float yaw) {
    setTranslation(tx, ty, tz);
    setRotation(roll, pitch, yaw);
}

// ������ƽ�Ʋ���
void HomogeneousTransform<true>::setTranslation(float x, float y, float z) {
    data[3] = x; data[7] = y; data[11] = z;
}

// ��������ת���֣�ʹ��ŷ���ǣ�roll, pitch, yaw��
void HomogeneousTransform<true>::setRotation(float roll, float pitch, float yaw) {
    float sr = sinf(roll),  cr = cosf(roll);
    float sp = sinf(pitch), cp = cosf(pitch);
    float sy = sinf(yaw),   cy = cosf(yaw);
    float tx = data[3], ty = data[7], tz = data[11]; // ���浱ǰƽ��

    // ������ת����Z-Y-X˳��
    data[0] = cy * cp;
    data[1] = cy * sp * sr - sy * cr;
    data[2] = cy * sp * cr + sy * sr;
    data[3] = tx; // �ָ�ƽ��

    data[4] = sy * cp;
    data[5] = sy * sp * sr + cy * cr;
    data[6] = sy * sp * cr - cy * sr;
    data[7] = ty; // �ָ�ƽ��

    data[8]  = -sp;
    data[9]  = cp * sr;
    data[10] = cp * cr;
    data[11] = tz; // �ָ�ƽ��
}

// Ӧ�ñ任������㣨�����������
void HomogeneousTransform<true>::apply(float x, float y, float z,
                                       float& outX, float& outY, float& outZ) const {
    outX = data[0]*x + data[1]*y + data[2]*z + data[3];
    outY = data[4]*x + data[5]*y + data[6]*z + data[7];
    outZ = data[8]*x + data[9]*y + data[10]*z + data[11];
}

// Ӧ�ñ任��Point3D���󣨷����µ㣩
Point3D HomogeneousTransform<true>::apply(const Point3D& p) const {
    return Point3D(
        data[0]*p.x + data[1]*p.y + data[2]*p.z + data[3],
        data[4]*p.x + data[5]*p.y + data[6]*p.z + data[7],
        data[8]*p.x + data[9]*p.y + data[10]*p.z + data[11]);
}

// ����˷�����ǰ�任����һ���任���
HomogeneousTransform<true>
HomogeneousTransform<true>::multiply(const HomogeneousTransform& o) const {
    HomogeneousTransform r;
    // 4x4����˷�
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += data[i*4+k] * o.data[k*4+j];
            r.data[i*4+j] = s;
        }
    return r;
}

// ����任������棨���ڸ���任�ļ򻯼��㣩
HomogeneousTransform<true>
HomogeneousTransform<true>::inverse() const {
    HomogeneousTransform r;
    // ��ת������������ת��
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.data[i*4+j] = data[j*4+i];
    // ����ƽ�Ʋ��ֵ��棺-R^T * t
    r.data[3]  = -(r.data[0]*data[3] + r.data[1]*data[7] + r.data[2]*data[11]);
    r.data[7]  = -(r.data[4]*data[3] + r.data[5]*data[7] + r.data[6]*data[11]);
    r.data[11] = -(r.data[8]*data[3] + r.data[9]*data[7] + r.data[10]*data[11]);
    r.data[15] = 1.0f; // ������겿��
    return r;
}

// ����ʾ����ʾ����
DemoResult runDemo()
{
    using namespace geometry;
    ReceiveRealData_S robotPos; // ������δ��ʼ����λ�ñ���
    
    // ��ʼ������ṹ��
    DemoResult r = {0};
    
    // 1. 2D�任��ʾ
    HomogeneousTransform<false> T1;
    T1.setTransform(1.0f, 1.0f, M_PI); // ������x��ƽ��1����y��ƽ��1����ת180��
    Point2D p2d(robotPos.x, robotPos.y); // ʹ��δ��ʼ����ֵ��ʵ��Ӧ����Ӧ�Ӷ��л�ȡ
    r.q2d = T1.apply(p2d); // Ӧ�ñ任

    // 2. 3D�任��ʾ����Z����ת90�ȣ���X��ƽ��1
    HomogeneousTransform<true> T2;
    T2.setTransform(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, M_PI/2);
    Point3D p3d(1.0f, 0.0f, 0.0f); // ��(1,0,0)
    r.q3d = T2.apply(p3d); // Ӧ�ñ任
    
    return r;
}

} // namespace geometry