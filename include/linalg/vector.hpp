#pragma once 

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "messages/geometry_msgs/vector3.hpp"

namespace linalg {

class Vector {
public:

    Vector();
    Vector(size_t size);
    Vector(size_t size, float val);
    Vector(const std::initializer_list<float>& data);
    Vector(const std::vector<float>& data);
    Vector(const Vector& v);
    Vector &operator=(const Vector& rhs);
    Vector(const geometry_msgs::Vector3& v);

    geometry_msgs::Vector3 toMsg() const;

    std::vector<float> getData() const;
    float get(size_t idx) const;
    float &at(size_t idx);
    void set(size_t idx, float val);
    size_t getSize() const;

    Vector operator-() const;
    Vector operator+(const Vector& rhs) const;
    Vector operator+(const float& rhs) const;
    Vector operator-(const Vector& rhs) const;
    Vector operator-(const float& rhs) const;
    Vector operator*(const Vector& rhs) const; // Element-wise multiplication
    Vector operator*(const float& rhs) const;
    Vector operator/(const Vector& rhs) const; // Element-wise division
    Vector operator/(const float& rhs) const;

    Vector operator+=(const Vector& rhs);
    Vector operator+=(const float& rhs);
    Vector operator-=(const Vector& rhs);
    Vector operator-=(const float& rhs);
    Vector operator*=(const Vector& rhs);
    Vector operator*=(const float& rhs);
    Vector operator/=(const Vector& rhs);
    Vector operator/=(const float& rhs);


    bool operator==(const Vector& rhs) const;
    bool operator!=(const Vector& rhs) const;

    static float dot(const Vector& lhs, const Vector& rhs);
    static Vector cross(const Vector& lhs, const Vector& rhs);
    static float norm(const Vector& v);
    static Vector normalize(const Vector& v);
    static float angle(const Vector& lhs, const Vector& rhs);
    static Vector LERP(const Vector& v1, const Vector& v2, float t);
    static Vector SLERP(const Vector& v1, const Vector& v2, float t);

protected:
    std::vector<float> data;
};

Vector operator+(const float& lhs, const Vector& rhs);

Vector operator-(const float& lhs, const Vector& rhs);

Vector operator*(const float& lhs, const Vector& rhs);

std::ostream &operator<<(std::ostream &os, const Vector& v);

} // namespace linalg