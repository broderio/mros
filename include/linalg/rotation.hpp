#pragma once

#include <iostream>
#include <cmath>
#include <vector>

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

namespace linalg
{

class Rotation : public Matrix 
{
public:
    Rotation();

    Rotation(const Matrix& m);

    static Rotation identity();
    static Rotation X(const float& angle);
    static Rotation Y(const float& angle);
    static Rotation Z(const float& angle);

    static Rotation fromQuaternion(const Quaternion& q);
    static Rotation fromRPY(const Vector& rpy);
    static Rotation fromAxisAngle(const Vector& axis, const float& angle);
    static Rotation fromEulerZYZ(const Vector& euler);
    static Rotation fromEulerZYX(const Vector& euler);

    static Rotation multiply(const Rotation &r, const linalg::Matrix &m);
    static Vector multiply(const Rotation &r, const linalg::Vector &v);
    static Rotation multiply(const Rotation& r1, const Rotation& r2);

    static Rotation inverse(const Rotation& r);

    Quaternion getQuaternion() const;
    Vector getRPY() const;
    void getAxisAngle(Vector &axis, float &angle) const;
    Vector getEulerZYZ() const;
    Vector getEulerZYX() const;
};

} // namespace linalg