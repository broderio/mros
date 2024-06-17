#ifndef QUATERNION_HPP
#define QUATERNION_HPP

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

    float norm() const;
    static Quaternion normalize(const Quaternion& q);
    static Quaternion multiply(const Quaternion& q1, const Quaternion& q2);
    static Matrix toRotationMatrix(const Quaternion& q);
    
    
private:
    float x;
    float y;
    float z;
    float w;
};

} // namespace linalg

#endif // QUATERNION_HPP