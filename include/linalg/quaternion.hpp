#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"

#include "messages/geometry_msgs/quaternion.hpp"

namespace linalg {

class Quaternion {
public:
    Quaternion();
    Quaternion(float x, float y, float z, float w);
    Quaternion(const geometry_msgs::Quaternion& q);
    Quaternion(const Quaternion& q);
    Quaternion &operator=(const Quaternion& rhs);
    Quaternion(const Vector& axis, const float& angle);

    geometry_msgs::Quaternion toMsg() const;

    Quaternion operator+(const Quaternion &rhs) const;
    Quaternion operator+(const float &rhs) const;
    Quaternion operator-(const Quaternion &rhs) const;
    Quaternion operator-(const float &rhs) const;
    Quaternion operator*(const float &rhs) const;
    Quaternion operator/(const float &rhs) const;

    float norm() const;
    Quaternion inverse() const;

    static Quaternion identity();
    static Quaternion fromRPY(const float& roll, const float& pitch, const float& yaw);
    static Quaternion normalize(const Quaternion& q);
    static float dot(const Quaternion& q1, const Quaternion& q2);
    static Quaternion multiply(const Quaternion& q1, const Quaternion& q2);
    static Vector rotate(const Vector &v, const Quaternion &q); // Passive rotation
    static Matrix toRotationMatrix(const Quaternion& q);
    static Quaternion SLERP(const Quaternion &q1, const Quaternion &q2, float t);
    
private:
    float x;
    float y;
    float z;
    float w;
};

Quaternion operator+(const float &lhs, const Quaternion &rhs);
Quaternion operator-(const float &lhs, const Quaternion &rhs);
Quaternion operator*(const float &lhs, const Quaternion &rhs);

} // namespace linalg