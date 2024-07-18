#include "linalg/quaternion.hpp"

namespace linalg 
{

Quaternion::Quaternion() : x(0), y(0), z(0), w(1) {}

Quaternion::Quaternion(const geometry_msgs::Quaternion &q) : x(q.x.data), y(q.y.data), z(q.z.data), w(q.w.data) {}

Quaternion::Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

Quaternion::Quaternion(const Quaternion &q) : x(q.x), y(q.y), z(q.z), w(q.w) {}

Quaternion &Quaternion::operator=(const Quaternion &rhs)
{
    if (this == &rhs)
    {
        return *this;
    }
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
    w = rhs.w;
    return *this;
}

Quaternion::Quaternion(const Vector &axis, const float &angle)
{
    float halfAngle = angle / 2;
    Vector axNorm = Vector::normalize(axis);
    
    x = axNorm.get(0) * std::sin(halfAngle);
    y = axNorm.get(1) * std::sin(halfAngle);
    z = axNorm.get(2) * std::sin(halfAngle);
    w = std::cos(halfAngle);
}

geometry_msgs::Quaternion Quaternion::toMsg() const
{
    return geometry_msgs::Quaternion(x, y, z, w);
}

Quaternion Quaternion::operator+(const Quaternion &rhs) const
{
    return Quaternion(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
}

Quaternion Quaternion::operator-(const Quaternion &rhs) const
{
    return Quaternion(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
}

Quaternion Quaternion::operator+(const float &rhs) const
{
    return Quaternion(x + rhs, y + rhs, z + rhs, w + rhs);
}

Quaternion Quaternion::operator-(const float &rhs) const
{
    return Quaternion(x - rhs, y - rhs, z - rhs, w - rhs);
}

Quaternion Quaternion::operator*(const float &rhs) const
{
    return Quaternion(x * rhs, y * rhs, z * rhs, w * rhs);
}

Quaternion Quaternion::operator/(const float &rhs) const
{
    return Quaternion(z / rhs, y / rhs, z / rhs, w / rhs);
}

float Quaternion::norm() const
{
    return std::sqrt(x * x + y * y + z * z + w * w);
}

Quaternion Quaternion::inverse() const
{
    return Quaternion(-x, -y, -z, w);
}

Quaternion Quaternion::fromRPY(const float &roll, const float &pitch, const float &yaw)
{
    float cy = std::cos(yaw * 0.5);
    float sy = std::sin(yaw * 0.5);
    float cp = std::cos(pitch * 0.5);
    float sp = std::sin(pitch * 0.5);
    float cr = std::cos(roll * 0.5);
    float sr = std::sin(roll * 0.5);

    float w = cr * cp * cy + sr * sp * sy;
    float x = sr * cp * cy - cr * sp * sy;
    float y = cr * sp * cy + sr * cp * sy;
    float z = cr * cp * sy - sr * sp * cy;

    return Quaternion(x, y, z, w);
}

Quaternion Quaternion::normalize(const Quaternion &q)
{
    float n = q.norm();
    return Quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
}

float Quaternion::dot(const Quaternion& q1, const Quaternion& q2)
{
    return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
}

Quaternion Quaternion::multiply(const Quaternion &q1, const Quaternion &q2)
{
    float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    return Quaternion(x, y, z, w);
}

Vector Quaternion::rotate(const Vector &v, const Quaternion &q)
{
    if (v.getSize() != 3) 
    {
        throw std::runtime_error("Vector must be size 3");
    }

    Quaternion p(v.get(0), v.get(1), v.get(2), 0);
    Quaternion pRot = multiply(multiply(q, p), q.inverse());

    Vector vRot(3);
    vRot.at(0) = pRot.x;
    vRot.at(1) = pRot.y;
    vRot.at(2) = pRot.z;
    return vRot;
}

Matrix Quaternion::toRotationMatrix(const Quaternion &q)
{
    float xx = q.x * q.x;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;
    float yy = q.y * q.y;
    float yz = q.y * q.z;
    float yw = q.y * q.w;
    float zz = q.z * q.z;
    float zw = q.z * q.w;
    return Matrix({{1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw)},
                   {2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw)},
                   {2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)}});
}

Quaternion Quaternion::SLERP(const Quaternion &q1, const Quaternion &q2, float t)
{
    float dot = Quaternion::dot(q1, q2);
    float theta = std::acos(dot);
    float s_theta = std::sqrt(1 - dot * dot); // sin(theta) == sin(acos(dot)) == sqrt(1 - dot^2)

    float s1 = std::sin(1 - t) * theta / s_theta;
    float s2 = std::sin(t * theta) / s_theta;

    return s1 * q1 + s2 * q2;
}

Quaternion operator+(const float &lhs, const Quaternion &rhs)
{
    return rhs + lhs;
}

Quaternion operator-(const float &lhs, const Quaternion &rhs)
{
    return (rhs * -1) + lhs;
}

Quaternion operator*(const float &lhs, const Quaternion &rhs)
{  
    return rhs * lhs;
}

} // namespace linalg