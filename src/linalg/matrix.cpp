#include "linalg/matrix.hpp"

using namespace linalg;

Matrix::Matrix()
: rows(), cols(), data() {}

Matrix::Matrix(size_t rows, size_t cols)
: rows(rows), cols(cols), data(rows, std::vector<double>(cols)) {}

Matrix::Matrix(size_t rows, size_t cols, double val)
: rows(rows), cols(cols), data(rows, std::vector<double>(cols, val)) {}

Matrix::Matrix(const std::vector<std::vector<double>>& data)
: rows(data.size()), cols(data[0].size()), data(data) {}

Matrix::Matrix(const Vector& v, bool column)
: rows(column ? v.getSize() : 1), cols(column ? 1 : v.getSize()), data(rows, std::vector<double>(cols)) {
    if (column) {
        std::vector<double> vData = v.getData();
        std::for_each(vData.begin(), vData.end(), [this](double f) {
            data.push_back({f});
        });
    } 
    else {
        data[0] = v.getData();
    } 
}

Matrix::Matrix(const Matrix& m)
: rows(m.rows), cols(m.cols), data(m.data) {}

Matrix &Matrix::operator=(const Matrix& rhs) {
    if (this == &rhs) {
        return *this;
    }
    rows = rhs.rows;
    cols = rhs.cols;
    data = rhs.data;
    return *this;
}


std::vector<std::vector<double>> Matrix::getData() const {
    return data;
}

double Matrix::get(size_t row, size_t col) const {
    if (row >= rows || col >= cols) {
        throw std::out_of_range("Index out of bounds.");
    }

    return data[row][col];
}

double &Matrix::at(size_t row, size_t col) {
    if (row >= rows || col >= cols) {
        throw std::out_of_range("Index out of bounds.");
    }

    return data[row][col];
}

void Matrix::set(size_t row, size_t col, double val) {
    if (row >= rows || col >= cols) {
        throw std::out_of_range("Index out of bounds.");
    }

    data[row][col] = val;
}

void Matrix::pushRow(const Vector &row)
{
    if (row.getSize() != getCols())
    {
        throw std::invalid_argument("Cannot push row of unequal size to matrix.");
    }
    data.push_back(row.getData());
    rows++;
}

void Matrix::pushCol(const Vector &col)
{
    if (col.getSize() != getRows())
    {
        throw std::invalid_argument("Cannot push column of unequal size to matrix.");
    }
    int i = 0;
    for (auto &row : data)
    {
        row.push_back(col.get(i++));
    }
    cols++;
}

Vector Matrix::getRow(size_t row) const {
    if (row >= rows) {
        throw std::out_of_range("Index out of bounds.");
    }

    return Vector(data[row]);
}

Vector Matrix::getCol(size_t col) const {
    if (col >= cols) {
        throw std::out_of_range("Index out of bounds.");
    }

    std::vector<double> v(rows);
    for (size_t i = 0; i < rows; ++i) {
        v[i] = data[i][col];
    }
    return Vector(v);
}

void Matrix::setRow(size_t row, const Vector& v) {
    if (row >= rows || v.getSize() != cols) {
        throw std::out_of_range("Index out of bounds or vector size mismatch.");
    }

    data[row] = v.getData();
}

void Matrix::setCol(size_t col, const Vector& v) {
    if (col >= cols || v.getSize() != rows) {
        throw std::out_of_range("Index out of bounds or vector size mismatch.");
    }

    for (size_t i = 0; i < rows; ++i) {
        data[i][col] = v.get(i);
    }
}

size_t Matrix::getRows() const {
    return rows;
}

size_t Matrix::getCols() const {
    return cols;
}

Matrix Matrix::getSubmatrix(size_t startRow, size_t startCol, size_t endRow, size_t endCol) const {
    if (startRow >= rows || startCol >= cols || endRow >= rows || endCol >= cols) {
        throw std::out_of_range("Index out of bounds.");
    }

    Matrix m(endRow - startRow + 1, endCol - startCol + 1);
    for (size_t i = startRow; i <= endRow; ++i) {
        for (size_t j = startCol; j <= endCol; ++j) {
            m.data[i - startRow][j - startCol] = data[i][j];
        }
    }
    return m;
}

Matrix Matrix::setSubmatrix(size_t startRow, size_t startCol, const Matrix& m) {
    if (startRow + m.rows > rows || startCol + m.cols > cols) {
        throw std::out_of_range("Index out of bounds.");
    }

    for (size_t i = startRow; i < startRow + m.rows; ++i) {
        for (size_t j = startCol; j < startCol + m.cols; ++j) {
            data[i][j] = m.data[i - startRow][j - startCol];
        }
    }
    return *this;
}

Matrix Matrix::operator+(const Matrix& rhs) const {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot add matrices of different sizes.");
    }

    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] + rhs.data[i][j];
        }
    }
    return m;
}

Matrix Matrix::operator+(const double& rhs) const {
    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] + rhs;
        }
    }
    return m;
}


Matrix Matrix::operator-(const Matrix& rhs) const {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot subtract matrices of different sizes.");
    }

    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] - rhs.data[i][j];
        }
    }
    return m;
}

Matrix Matrix::operator-(const double& rhs) const {
    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] - rhs;
        }
    }
    return m;
}


Matrix Matrix::operator*(const Matrix& rhs) const {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot multiply matrices of different sizes.");
    }

    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] * rhs.data[i][j];
        }
    }
    return m;
}

Matrix Matrix::operator*(const double& rhs) const {
    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] * rhs;
        }
    }
    return m;
}


Matrix Matrix::operator/(const Matrix& rhs) const {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot divide matrices of different sizes.");
    }

    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] / rhs.data[i][j];
        }
    }
    return m;
}

Matrix Matrix::operator/(const double& rhs) const {
    Matrix m(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            m.data[i][j] = data[i][j] / rhs;
        }
    }
    return m;
}


Matrix Matrix::operator+=(const Matrix& rhs) {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot add matrices of different sizes.");
    }

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] += rhs.data[i][j];
        }
    }
    return *this;
}

Matrix Matrix::operator+=(const double& rhs) {
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] += rhs;
        }
    }
    return *this;
}


Matrix Matrix::operator-=(const Matrix& rhs) {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot subtract matrices of different sizes.");
    }

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] -= rhs.data[i][j];
        }
    }
    return *this;
}

Matrix Matrix::operator-=(const double& rhs) {
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] -= rhs;
        }
    }
    return *this;
}


Matrix Matrix::operator*=(const Matrix& rhs) {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot multiply matrices of different sizes.");
    }

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] *= rhs.data[i][j];
        }
    }
    return *this;
}

Matrix Matrix::operator*=(const double& rhs) {
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] *= rhs;
        }
    }
    return *this;
}


Matrix Matrix::operator/=(const Matrix& rhs) {
    if (rows != rhs.rows || cols != rhs.cols) {
        throw std::invalid_argument("Cannot divide matrices of different sizes.");
    }

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] /= rhs.data[i][j];
        }
    }
    return *this;
}

Matrix Matrix::operator/=(const double& rhs) {
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            data[i][j] /= rhs;
        }
    }
    return *this;
}


Matrix operator+(const double& lhs, const Matrix& rhs) {
    Matrix m(rhs.getRows(), rhs.getCols());
    for (size_t i = 0; i < rhs.getRows(); ++i) {
        for (size_t j = 0; j < rhs.getCols(); ++j) {
            m.at(i, j) = lhs + rhs.get(i, j);
        }
    }
    return m;
}

Matrix operator-(const double& lhs, const Matrix& rhs) {
    Matrix m(rhs.getRows(), rhs.getCols());
    for (size_t i = 0; i < rhs.getRows(); ++i) {
        for (size_t j = 0; j < rhs.getCols(); ++j) {
            m.at(i, j) = lhs - rhs.get(i, j);
        }
    }
    return m;
}


Matrix Matrix::identity(size_t size) {
    Matrix m(size, size);
    for (size_t i = 0; i < size; ++i) {
        m.data[i][i] = 1;
    }
    return m;
}

Matrix Matrix::multiply(const Vector& colVector, const Vector& rowVector) {
    Matrix m(colVector.getSize(), rowVector.getSize());
    for (size_t i = 0; i < colVector.getSize(); ++i) {
        for (size_t j = 0; j < rowVector.getSize(); ++j) {
            m.data[i][j] = colVector.get(i) * rowVector.get(j);
        }
    }
    return m;
}

Vector Matrix::multiply(const Matrix& m, const Vector& colVector) {
    if (m.getCols() != colVector.getSize()) {
        throw std::invalid_argument("Cannot multiply matrix and vector of incompatible sizes.");
    }

    Vector v(m.getRows());
    for (size_t i = 0; i < m.getRows(); ++i) {
        for (size_t j = 0; j < m.getCols(); ++j) {
            v.at(i) += m.data[i][j] * colVector.get(j);
        }
    }
    return v;
}

Matrix Matrix::multiply(const Matrix& lhs, const Matrix& rhs) {
    if (lhs.getCols() != rhs.getRows()) {
        throw std::invalid_argument("Cannot multiply matrices of incompatible sizes.");
    }

    Matrix m(lhs.getRows(), rhs.getCols());
    for (size_t i = 0; i < lhs.getRows(); ++i) {
        for (size_t j = 0; j < rhs.getCols(); ++j) {
            for (size_t k = 0; k < lhs.getCols(); ++k) {
                m.data[i][j] += lhs.data[i][k] * rhs.data[k][j];
            }
        }
    }
    return m;
}

Matrix Matrix::transpose(const Matrix& m) {
    Matrix t(m.cols, m.rows);
    for (size_t i = 0; i < m.rows; ++i) {
        for (size_t j = 0; j < m.cols; ++j) {
            t.data[j][i] = m.data[i][j];
        }
    }
    return t;
}

bool Matrix::LU(const Matrix& A, Matrix& L, Matrix& U) {
    if (A.rows != A.cols) {
        throw std::invalid_argument("Cannot factorize non-square matrix.");
    }

    Matrix temp(A);
    L = Matrix(A.rows, A.cols);
    U = Matrix(A.rows, A.cols);

    for (size_t i = 0; i < A.rows; ++i) 
    {

        Vector c = temp.getCol(i);
        Vector r = temp.getRow(i);
        double pivot = c.get(i);

        if (!approx(pivot, 0)) 
        {
            c /= pivot;
            temp -= multiply(c, r);
            L.setCol(i, c);
            U.setRow(i, r);
        }
        else 
        {
            std::cerr << "Error: cannot factorize singular matrix." << std::endl;
            return false;
        }
    }
    return true;
}

bool Matrix::QR(const Matrix& A, Matrix& Q, Matrix& R) {
    if (A.rows != A.cols) {
        throw std::invalid_argument("Cannot factorize non-square matrix.");
    }

    Q = Matrix(A.rows, A.cols);

    for (size_t i = 0; i < A.cols; ++i) 
    {
        Vector sum(A.rows);
        Vector u = A.getCol(i);
        for (size_t j = 0; j < i; ++j) 
        {
            Vector q = Q.getCol(j);
            u -= q * (Vector::dot(u, q) / Vector::dot(q, q));
        }
        Q.setCol(i, Vector::normalize(u));
    }
    R = multiply(transpose(Q), A);
    
    return true;
}

Matrix Matrix::inverse(const Matrix& m) {
    if (m.rows != m.cols) {
        throw std::invalid_argument("Cannot invert non-square matrix.");
    }

    // Inversion using LU decomposition

    Matrix L, U;
    if (!LU(m, L, U)) {
        return Matrix();
    }

    Matrix Uinv = invertUpperTriangular(U);
    Matrix Linv = invertLowerTriangular(L);

    Matrix inv = multiply(Uinv, Linv);

    return inv;
}

Matrix Matrix::pseudoInverse(const Matrix& m) {
    if (m.rows == m.cols) {
        return inverse(m);
    }
    else if (m.rows > m.cols) {
        Matrix mt = transpose(m);
        return multiply(inverse(multiply(mt, m)), mt);
    }
    else {
        Matrix mt = transpose(m);
        return multiply(mt, inverse(multiply(m, mt)));
    }
}

Matrix Matrix::invertUpperTriangular(const Matrix& U) {
    if (U.rows != U.cols) {
        throw std::invalid_argument("Cannot invert non-square matrix.");
    }

    Matrix inv(U.rows, U.cols);
    for (size_t i = 0; i < U.rows; ++i) {
        inv.data[i][i] = 1 / U.data[i][i];
        for (size_t j = i + 1; j < U.cols; ++j) {
            double sum = 0;
            for (size_t k = i; k < j; ++k) {
                sum += U.data[k][j] * inv.data[i][k];
            }
            inv.data[i][j] = -sum / U.data[j][j];
        }
    }
    return inv;
}

Matrix Matrix::invertLowerTriangular(const Matrix& L) {
    if (L.rows != L.cols) {
        throw std::invalid_argument("Cannot invert non-square matrix.");
    }

    Matrix inv(L.rows, L.cols);
    for (size_t i = 0; i < L.rows; ++i) {
        inv.data[i][i] = 1 / L.data[i][i];
        for (size_t j = 0; j < i; ++j) {
            double sum = 0;
            for (size_t k = j; k < i; ++k) {
                sum += L.data[i][k] * inv.data[k][j];
            }
            inv.data[i][j] = -sum / L.data[i][i];
        }
    }
    return inv;
}

Vector Matrix::backwardSub(const Matrix& U, const Vector& b) {
    if (U.rows != U.cols || U.rows != b.getSize()) {
        throw std::invalid_argument("Cannot perform backward substitution on incompatible sizes.");
    }

    Vector x(U.rows);
    for (int i = U.rows - 1; i >= 0; --i) {
        double sum = 0;
        for (size_t j = i + 1; j < U.cols; ++j) {
            sum += U.data[i][j] * x.get(j);
        }
        x.at(i) = (b.get(i) - sum) / U.data[i][i];
    }
    return x;
}

Vector Matrix::forwardSub(const Matrix& L, const Vector& b) {
    if (L.rows != L.cols || L.rows != b.getSize()) {
        throw std::invalid_argument("Cannot perform forward substitution on incompatible sizes.");
    }

    Vector x(L.rows);
    for (size_t i = 0; i < L.rows; ++i) {
        double sum = 0;
        for (size_t j = 0; j < i; ++j) {
            sum += L.data[i][j] * x.get(j);
        }
        x.at(i) = (b.get(i) - sum) / L.data[i][i];
    }
    return x;
}

Matrix linalg::operator+(const double& lhs, const Matrix& rhs) {
    Matrix m(rhs.getRows(), rhs.getCols());
    for (size_t i = 0; i < rhs.getRows(); ++i) {
        for (size_t j = 0; j < rhs.getCols(); ++j) {
            m.at(i, j) = rhs.get(i, j) + lhs;
        }
    }
    return m;
}

Matrix linalg::operator-(const double& lhs, const Matrix& rhs) {
    Matrix m(rhs.getRows(), rhs.getCols());
    for (size_t i = 0; i < rhs.getRows(); ++i) {
        for (size_t j = 0; j < rhs.getCols(); ++j) {
            m.at(i, j) = rhs.get(i, j) - lhs;
        }
    }
    return m;
}

Matrix linalg::operator*(const double& lhs, const Matrix& rhs) {
    Matrix m(rhs.getRows(), rhs.getCols());
    for (size_t i = 0; i < rhs.getRows(); ++i) {
        for (size_t j = 0; j < rhs.getCols(); ++j) {
            m.at(i, j) = lhs * rhs.get(i, j);
        }
    }
    return m;
}

std::ostream &linalg::operator<<(std::ostream& os, const Matrix& m) {
    os << "[";
    for (size_t i = 0; i < m.getRows(); ++i) {
        os << "[";
        for (size_t j = 0; j < m.getCols(); ++j) {
            os << m.get(i, j);
            if (j < m.getCols() - 1) {
                os << ", ";
            }
        }
        os << "]";
        if (i < m.getRows() - 1) {
            os << ",\n";
        }
    }
    os << "]";
    return os;
}

bool linalg::approx(const double& lhs, const double& rhs, const double& eps) {
    return std::abs(lhs - rhs) < eps;
}

Vector linalg::linearSolve(Matrix A, const Vector& b, const bool& useLU) {
    Matrix At = Matrix::transpose(A);
    if (A.getRows() != A.getCols()) {
        A = Matrix::multiply(At, A);
    }

    if (useLU) {
        Matrix L, U;
        if (!Matrix::LU(A, L, U)) {
            return Vector();
        }

        Vector y = Matrix::forwardSub(L, Matrix::multiply(At, b));
        return Matrix::backwardSub(U, y);
    }
    else {
        // Use QR (this is less numerically stable than LU, but more robust to singular matrices)
        Matrix Q, R;
        if (!Matrix::QR(A, Q, R)) {
            return Vector();
        }

        return Matrix::backwardSub(R, Matrix::multiply(Matrix::transpose(Q), Matrix::multiply(At, b)));
    }
}

Vector linalg::polyfit(const Vector& x, const Vector& y, const size_t& order, const bool& useLU) {
    if (x.getSize() != y.getSize()) {
        throw std::invalid_argument("Cannot fit polynomial to vectors of different sizes.");
    }

    Matrix A(x.getSize(), order + 1);
    for (size_t i = 0; i < x.getSize(); ++i) {
        for (size_t j = 0; j <= order; ++j) {
            A.at(i, j) = std::pow(x.get(i), j);
        }
    }

    return linalg::linearSolve(A, y, useLU);
}