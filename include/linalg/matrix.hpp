#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include "linalg/vector.hpp"

namespace linalg {

class Matrix {
public:
    
    Matrix();
    Matrix(size_t rows, size_t cols);
    Matrix(size_t rows, size_t cols, double val);
    Matrix(const std::vector<std::vector<double>>& data);
    Matrix(const Vector& v, bool column = true);
    Matrix(const Matrix& m);

    Matrix &operator=(const Matrix& rhs);

    std::vector<std::vector<double>> getData() const;

    double get(size_t row, size_t col) const;
    double &at(size_t row, size_t col);
    void set(size_t row, size_t col, double val);

    void pushRow(const Vector &row);
    void pushCol(const Vector &col);

    Vector getRow(size_t row) const;
    Vector getCol(size_t col) const;

    void setRow(size_t row, const Vector& v);
    void setCol(size_t col, const Vector& v);

    size_t getRows() const;
    size_t getCols() const;

    Matrix getSubmatrix(size_t startRow, size_t startCol, size_t endRow, size_t endCol) const;
    Matrix setSubmatrix(size_t startRow, size_t startCol, const Matrix& m);

    Matrix operator+(const Matrix& rhs) const;
    Matrix operator+(const double& rhs) const;

    Matrix operator-(const Matrix& rhs) const;
    Matrix operator-(const double& rhs) const;

    Matrix operator*(const Matrix& rhs) const; // Element-wise multiplication
    Matrix operator*(const double& rhs) const;

    Matrix operator/(const Matrix& rhs) const; // Element-wise division
    Matrix operator/(const double& rhs) const;

    Matrix operator+=(const Matrix& rhs);
    Matrix operator+=(const double& rhs);

    Matrix operator-=(const Matrix& rhs);
    Matrix operator-=(const double& rhs);

    Matrix operator*=(const Matrix& rhs);
    Matrix operator*=(const double& rhs);

    Matrix operator/=(const Matrix& rhs);
    Matrix operator/=(const double& rhs);

    static Matrix identity(size_t size);

    static Matrix multiply(const Vector& colVector, const Vector& rowVector);
    static Vector multiply(const Matrix& m, const Vector& colVector);
    static Matrix multiply(const Matrix& lhs, const Matrix& rhs);

    static Matrix transpose(const Matrix& m);

    static bool LU(const Matrix& m, Matrix& L, Matrix& U);
    static bool QR(const Matrix& m, Matrix& Q, Matrix& R);

    static Matrix invertUpperTriangular(const Matrix& U);
    static Matrix invertLowerTriangular(const Matrix& L);
    static Matrix inverse(const Matrix& m);
    static Matrix pseudoInverse(const Matrix& m);
    static Vector backwardSub(const Matrix& U, const Vector& b);
    static Vector forwardSub(const Matrix& L, const Vector& b);

protected:

    size_t rows;

    size_t cols;

    std::vector<std::vector<double>> data;

};


Matrix operator+(const double& lhs, const Matrix& rhs);

Matrix operator-(const double& lhs, const Matrix& rhs);

Matrix operator*(const double& lhs, const Matrix& rhs);

std::ostream &operator<<(std::ostream& os, const Matrix& m);

bool approx(const double& lhs, const double& rhs, const double& eps = 1e-6);

Vector linearSolve(Matrix A, const Vector& b, const bool& useLU = true);

Vector polyfit(const Vector& x, const Vector& y, const size_t& order, const bool& useLU = true);

} // namespace linalg