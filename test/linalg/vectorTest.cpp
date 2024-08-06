#include <iostream>
#include <vector>
#include <string>

#include "linalg/vector.hpp"

using namespace linalg;

int main() {
    std::vector<double> data = {0, 0, 1};
    Vector v1(data);
    data[0] = 1;
    data[2] = 0;
    Vector v2(data);

    Vector v3 = v1 + v2;
    Vector v4 = v1 - v2;
    Vector v5 = v1 * v2;
    double dot = Vector::dot(v1, v2);
    Vector cross = Vector::cross(v1, v2);
    double norm = Vector::norm(cross);
    double angle = Vector::angle(v1, v2);

    std::cout << "v1: " << v1 << std::endl;
    std::cout << "v2 " << v2 << std::endl;
    std::cout << "v1 + v2: " << v3 << std::endl;
    std::cout << "v1 - v2: " << v4 << std::endl;
    std::cout << "v1 * v2: " << v5 << std::endl;
    std::cout << "dot(v1, v2): " << dot << std::endl;
    std::cout << "cross(v1, v2): " << cross << std::endl;
    std::cout << "norm(cross(v1, v2)): " << norm << std::endl;
    std::cout << "angle(v1, v2): " << angle << std::endl;

    return 0;
}