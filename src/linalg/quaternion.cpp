#include "linalg/quaternion.hpp"

using namespace linalg;

Quaternion::Quaternion() : x(0), y(0), z(0), w(1) {}

Quaternion::Quaternion(const geometry_msgs::Quaternion& q) : x(q.x.data), y(q.y.data), z(q.z.data), w(q.w.data) {}

Quaternion::Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

Quaternion::Quaternion(const Quaternion& q) : x(q.x), y(q.y), z(q.z), w(q.w) {}

Quaternion &Quaternion::operator=(const Quaternion& rhs) {
    if (this == &rhs) {
        return *this;
    }
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
    w = rhs.w;
    return *this;
}

Quaternion::Quaternion(const Vector& axis, const float& angle) {
    float halfAngle = angle / 2;
    x = std::cos(halfAngle);
    y = axis.get(0) * std::sin(halfAngle);
    z = axis.get(1) * std::sin(halfAngle);
    w = axis.get(2) * std::sin(halfAngle);
}

geometry_msgs::Quaternion Quaternion::toMsg() const {
    return geometry_msgs::Quaternion(x, y, z, w);
}

float Quaternion::norm() const {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

Quaternion Quaternion::normalize(const Quaternion& q) {
    float n = q.norm();
    return Quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
}

Quaternion Quaternion::multiply(const Quaternion& q1, const Quaternion& q2) {
    float x = q1.x * q2.x - q1.y * q2.y - q1.z * q2.z - q1.w * q2.w;
    float y = q1.x * q2.y + q1.y * q2.x + q1.z * q2.w - q1.w * q2.z;
    float z = q1.x * q2.z - q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    float w = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    return Quaternion(x, y, z, w);
}

Matrix Quaternion::toRotationMatrix(const Quaternion& q) {
    float xx = q.x * q.x;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;
    float yy = q.y * q.y;
    float yz = q.y * q.z;
    float yw = q.y * q.w;
    float zz = q.z * q.z;
    float zw = q.z * q.w;
    return Matrix({{1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), 0},
                   {2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw), 0},
                   {2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy), 0},
                   {0,             0,             0,                 1}});
}