#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <iostream>
#include <vector>
#include <cmath>

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"

class Quaternion {
public:
    Quaternion();
    
    Quaternion(float x, float y, float z, float w);

    Quaternion(const Quaternion& q);

    Quaternion &operator=(const Quaternion& rhs);

    Quaternion(const Vector& axis, const float& angle);

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

#endif // QUATERNION_HPP