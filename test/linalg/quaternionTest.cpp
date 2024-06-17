#include <iostream>
#include <vector>

#include "utils.hpp"
#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"


int main() {
    Vector axis(3);
    axis.at(0) = 1;
    axis.at(1) = 0;
    axis.at(2) = 0;
    Quaternion q(axis, degToRad(90));
    Matrix rot = Quaternion::toRotationMatrix(q);
    std::cout << "Rotation matrix:\n" << rot << std::endl;
}