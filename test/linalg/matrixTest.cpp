#include <iostream>
#include <vector>
#include <string>

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"

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

    Matrix A(10, 2);
    for (int i = 0; i < 10; i++) {
        A.at(i, 0) = 1;
        A.at(i, 1) = x.get(i);
    }

    Matrix At = Matrix::transpose(A);
    Matrix AtA = Matrix::multiply(At, A);
    Vector xStarLU = Matrix::solve(AtA, Matrix::multiply(At, y));
    Vector xStarQR = Matrix::solve(AtA, Matrix::multiply(At, y), false);

    std::cout << "LU Linear fit: y = " << xStarLU.get(0) << " + " << xStarLU.get(1) << "x" << std::endl;
    std::cout << "QR Linear fit: y = " << xStarQR.get(0) << " + " << xStarQR.get(1) << "x" << std::endl;
}