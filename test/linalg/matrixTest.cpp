#include <iostream>
#include <vector>
#include <string>

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"

using namespace linalg;

int main() {
    // Linear fit test
    Vector x(10);
    for (int i = 0; i < 10; i++) {
        x.at(i) = i;
    }

    Vector y(10);
    y.set(0, 2);
    y.set(1, 4);
    y.set(2, 3);
    y.set(3, 5);
    y.set(4, 8);
    y.set(5, 6);
    y.set(6, 7);
    y.set(7, 9);
    y.set(8, 11);
    y.set(9, 10);

    Vector xStarLU = polyfit(x, y, 1);
    Vector xStarQR = polyfit(x, y, 1, false);

    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;

    std::cout << "LU Linear fit: y = " << xStarLU.get(0) << " + " << xStarLU.get(1) << "x" << std::endl;
    std::cout << "QR Linear fit: y = " << xStarQR.get(0) << " + " << xStarQR.get(1) << "x" << std::endl;

    Vector y2(10);
    y2.set(0, 16);
    y2.set(1, 18);
    y2.set(2, 21);
    y2.set(3, 22);
    y2.set(4, 23);
    y2.set(5, 22);
    y2.set(6, 20);
    y2.set(7, 19);
    y2.set(8, 16);
    y2.set(9, 10);

    Vector xStarLU2 = polyfit(x, y2, 2);
    Vector xStarQR2 = polyfit(x, y2, 2, false);

    std::cout << "\nx: " << x << std::endl;
    std::cout << "y: " << y2 << std::endl;

    std::cout << "LU Quadratic fit: y = " << xStarLU2.get(0) << " + " << xStarLU2.get(1) << "x + " << xStarLU2.get(2) << "x^2" << std::endl;
    std::cout << "QR Quadratic fit: y = " << xStarQR2.get(0) << " + " << xStarQR2.get(1) << "x + " << xStarQR2.get(2) << "x^2" << std::endl;

    // Test inversion
    Matrix m(3, 3);
    m.at(0, 0) = 1;
    m.at(0, 1) = 2;
    m.at(0, 2) = 3;
    m.at(1, 0) = 0;
    m.at(1, 1) = 1;
    m.at(1, 2) = 4;
    m.at(2, 0) = 5;
    m.at(2, 1) = 6;
    m.at(2, 2) = 0;

    Matrix mInv = Matrix::inverse(m);

    std::cout << "\nMatrix:\n" << m << std::endl;
    std::cout << "\nInverse:\n" << mInv << std::endl;
    std::cout << "\nm * mInv:\n" << Matrix::multiply(m, mInv) << std::endl;
}