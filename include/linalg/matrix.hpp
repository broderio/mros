#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "linalg/vector.hpp"

namespace linalg {

class Matrix {
public:
    
    Matrix();
    Matrix(size_t rows, size_t cols);
    Matrix(size_t rows, size_t cols, float val);
    Matrix(const std::vector<std::vector<float>>& data);
    Matrix(const Vector& v, bool column = true);
    Matrix(const Matrix& m);

    Matrix &operator=(const Matrix& rhs);

    std::vector<std::vector<float>> getData() const;
    float get(size_t row, size_t col) const;
    float &at(size_t row, size_t col);
    void set(size_t row, size_t col, float val);
    Vector getRow(size_t row) const;
    Vector getCol(size_t col) const;
    void setRow(size_t row, const Vector& v);
    void setCol(size_t col, const Vector& v);
    size_t getRows() const;
    size_t getCols() const;

    Matrix operator+(const Matrix& rhs) const;
    Matrix operator+(const float& rhs) const;

    Matrix operator-(const Matrix& rhs) const;
    Matrix operator-(const float& rhs) const;

    Matrix operator*(const Matrix& rhs) const; // Element-wise multiplication
    Matrix operator*(const float& rhs) const;

    Matrix operator/(const Matrix& rhs) const; // Element-wise division
    Matrix operator/(const float& rhs) const;

    Matrix operator+=(const Matrix& rhs);
    Matrix operator+=(const float& rhs);

    Matrix operator-=(const Matrix& rhs);
    Matrix operator-=(const float& rhs);

    Matrix operator*=(const Matrix& rhs);
    Matrix operator*=(const float& rhs);

    Matrix operator/=(const Matrix& rhs);
    Matrix operator/=(const float& rhs);

    static Matrix identity(size_t size);
    static Matrix translation(float x, float y, float z);
    static Matrix rotationX(float angle);
    static Matrix rotationY(float angle);
    static Matrix rotationZ(float angle);

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

private:

    size_t rows;

    size_t cols;

    std::vector<std::vector<float>> data;

};


Matrix operator+(const float& lhs, const Matrix& rhs);

Matrix operator-(const float& lhs, const Matrix& rhs);

Matrix operator*(const float& lhs, const Matrix& rhs);

std::ostream &operator<<(std::ostream& os, const Matrix& m);

bool approx(const float& lhs, const float& rhs, const float& eps = 1e-6);

Vector linearSolve(const Matrix& A, const Vector& b, const bool& useLU = true);

Vector polyfit(const Vector& x, const Vector& y, const size_t& order, const bool& useLU = true);

} // namespace linalg

#endif // MATRIX_HPP