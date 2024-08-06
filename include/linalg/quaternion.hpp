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
    Quaternion(double x, double y, double z, double w);
    Quaternion(const geometry_msgs::Quaternion& q);
    Quaternion(const Quaternion& q);
    Quaternion &operator=(const Quaternion& rhs);
    Quaternion(const Vector& axis, const double& angle);

    geometry_msgs::Quaternion toMsg() const;
    void toAxisAngle(Vector& axis, double& angle) const;

    Quaternion operator+(const Quaternion &rhs) const;
    Quaternion operator+(const double &rhs) const;
    Quaternion operator-(const Quaternion &rhs) const;
    Quaternion operator-(const double &rhs) const;
    Quaternion operator*(const double &rhs) const;
    Quaternion operator/(const double &rhs) const;

    double norm() const;
    Quaternion inverse() const;

    static Quaternion identity();
    static Quaternion fromRPY(const double& roll, const double& pitch, const double& yaw);
    static Quaternion normalize(const Quaternion& q);
    static double dot(const Quaternion& q1, const Quaternion& q2);
    static Quaternion multiply(const Quaternion& q1, const Quaternion& q2);
    static Vector rotate(const Vector &v, const Quaternion &q); // Passive rotation
    static Matrix toRotationMatrix(const Quaternion& q);
    static Quaternion SLERP(const Quaternion &q1, const Quaternion &q2, double t);
    
private:
    double x;
    double y;
    double z;
    double w;
};

Quaternion operator+(const double &lhs, const Quaternion &rhs);
Quaternion operator-(const double &lhs, const Quaternion &rhs);
Quaternion operator*(const double &lhs, const Quaternion &rhs);

} // namespace linalg