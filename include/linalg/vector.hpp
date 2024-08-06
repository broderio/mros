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
    Vector(size_t size, double val);
    Vector(const std::initializer_list<double>& data);
    Vector(const std::vector<double>& data);
    Vector(const Vector& v);
    Vector &operator=(const Vector& rhs);
    Vector(const geometry_msgs::Vector3& v);

    geometry_msgs::Vector3 toMsg() const;

    std::vector<double> getData() const;
    double get(size_t idx) const;
    double &at(size_t idx);
    void set(size_t idx, double val);
    size_t getSize() const;

    void push_back(double val);
    void pop_back();
    Vector subvec(size_t start, size_t size) const;

    Vector operator-() const;
    Vector operator+(const Vector& rhs) const;
    Vector operator+(const double& rhs) const;
    Vector operator-(const Vector& rhs) const;
    Vector operator-(const double& rhs) const;
    Vector operator*(const Vector& rhs) const; // Element-wise multiplication
    Vector operator*(const double& rhs) const;
    Vector operator/(const Vector& rhs) const; // Element-wise division
    Vector operator/(const double& rhs) const;

    Vector operator+=(const Vector& rhs);
    Vector operator+=(const double& rhs);
    Vector operator-=(const Vector& rhs);
    Vector operator-=(const double& rhs);
    Vector operator*=(const Vector& rhs);
    Vector operator*=(const double& rhs);
    Vector operator/=(const Vector& rhs);
    Vector operator/=(const double& rhs);


    bool operator==(const Vector& rhs) const;
    bool operator!=(const Vector& rhs) const;

    static double dot(const Vector& lhs, const Vector& rhs);
    static Vector cross(const Vector& lhs, const Vector& rhs);
    static double norm(const Vector& v);
    static Vector normalize(const Vector& v);
    static double angle(const Vector& lhs, const Vector& rhs);
    static Vector LERP(const Vector& v1, const Vector& v2, double t);
    static Vector SLERP(const Vector& v1, const Vector& v2, double t);

protected:
    std::vector<double> data;
};

Vector operator+(const double& lhs, const Vector& rhs);

Vector operator-(const double& lhs, const Vector& rhs);

Vector operator*(const double& lhs, const Vector& rhs);

std::ostream &operator<<(std::ostream &os, const Vector& v);

} // namespace linalg