#include "linalg/vector.hpp"

using namespace linalg;

Vector::Vector() : data() {}

Vector::Vector(size_t size) : data(size) {}

Vector::Vector(size_t size, double val) : data(size, val) {}

Vector::Vector(const std::initializer_list<double>& data) : data(data) {}

Vector::Vector(const Vector& v) : data(v.data) {}

Vector &Vector::operator=(const Vector& rhs) {
    if (this == &rhs) {
        return *this;
    }
    data = rhs.data;
    return *this;
}

Vector::Vector(const std::vector<double>& data) : data(data) {}

Vector::Vector(const geometry_msgs::Vector3& v) : data({v.x.data, v.y.data, v.z.data}) {}

geometry_msgs::Vector3 Vector::toMsg() const {
    return geometry_msgs::Vector3(data[0], data[1], data[2]);
}

std::vector<double> Vector::getData() const {
    return data;
}

double Vector::get(size_t idx) const {
    if (idx >= data.size()) {
        throw std::out_of_range("Index out of bounds.");
    }

    return data[idx];
}

double &Vector::at(size_t idx) {
    if (idx >= data.size()) {
        throw std::out_of_range("Index out of bounds.");
    }

    return data[idx];
}

void Vector::set(size_t idx, double val) {
    if (idx >= data.size()) {
        throw std::out_of_range("Index out of bounds.");
    }

    data[idx] = val;
}

size_t Vector::getSize() const {
    return data.size();
}

void Vector::push_back(double val) {
    data.push_back(val);
}

void Vector::pop_back() {
    data.pop_back();
}

Vector Vector::subvec(size_t start, size_t size) const {
    if (start >= data.size() || (start + size - 1) >= data.size()) {
        throw std::out_of_range("Index out of bounds.");
    }

    return Vector(std::vector<double>(data.begin() + start, data.begin() + start + size));
}

Vector Vector::operator-() const {
    Vector v(*this);
    std::for_each(v.data.begin(), v.data.end(), [](double& f) { f = -f; });
    return v;
}


Vector Vector::operator+(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot add vectors of different sizes.");
    }

    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] + rhs.data[i];
    }
    return v;
}

Vector Vector::operator+(const double& rhs) const {
    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] + rhs;
    }
    return v;
}


Vector Vector::operator-(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot subtract vectors of different sizes.");
    }

    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] - rhs.data[i];
    }
    return v;
}

Vector Vector::operator-(const double& rhs) const {
    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] - rhs;
    }
    return v;
}


Vector Vector::operator*(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot multiply vectors of different sizes.");
    }

    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] * rhs.data[i];
    }
    return v;
}

Vector Vector::operator*(const double& rhs) const {
    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] * rhs;
    }
    return v;
}


Vector Vector::operator/(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot divide vectors of different sizes.");
    }

    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] / rhs.data[i];
    }
    return v;
}

Vector Vector::operator/(const double& rhs) const {
    Vector v(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        v.data[i] = data[i] / rhs;
    }
    return v;
}


Vector Vector::operator+=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot add vectors of different sizes.");
    }

    for (size_t i = 0; i < data.size(); ++i) {
        data[i] += rhs.data[i];
    }
    return *this;
}

Vector Vector::operator+=(const double& rhs) {
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] += rhs;
    }
    return *this;
}


Vector Vector::operator-=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot subtract vectors of different sizes.");
    }

    for (size_t i = 0; i < data.size(); ++i) {
        data[i] -= rhs.data[i];
    }
    return *this;
}

Vector Vector::operator-=(const double& rhs) {
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] -= rhs;
    }
    return *this;
}


Vector Vector::operator*=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot multiply vectors of different sizes.");
    }

    for (size_t i = 0; i < data.size(); ++i) {
        data[i] *= rhs.data[i];
    }
    return *this;
}

Vector Vector::operator*=(const double& rhs) {
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] *= rhs;
    }
    return *this;
}


Vector Vector::operator/=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        throw std::invalid_argument("Cannot divide vectors of different sizes.");
    }

    for (size_t i = 0; i < data.size(); ++i) {
        data[i] /= rhs.data[i];
    }
    return *this;
}

Vector Vector::operator/=(const double& rhs) {
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] /= rhs;
    }
    return *this;
}


bool Vector::operator==(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        return false;
    }

    for (size_t i = 0; i < data.size(); ++i) {
        if (data[i] != rhs.data[i]) {
            return false;
        }
    }

    return true;
}

bool Vector::operator!=(const Vector& rhs) const {
    return !(*this == rhs);
}


Vector linalg::operator+(const double& lhs, const Vector& rhs) {
    return rhs + lhs;
}

Vector linalg::operator-(const double& lhs, const Vector& rhs) {
    return -rhs + lhs;
}

Vector linalg::operator*(const double& lhs, const Vector& rhs) {
    return rhs * lhs;
}


double Vector::dot(const Vector& lhs, const Vector& rhs) {
    if (lhs.getSize() != rhs.getSize()) {
        throw std::invalid_argument("Cannot take dot product of vectors of different sizes.");
    }

    double result = 0;
    for (size_t i = 0; i < lhs.getSize(); ++i) {
        result += lhs.get(i) * rhs.get(i);
    }

    return result;
}

Vector Vector::cross(const Vector& lhs, const Vector& rhs) {
    if (lhs.getSize() != 3 || rhs.getSize() != 3) {
        throw std::invalid_argument("Cross product is only defined for 3D vectors.");
    }

    std::vector<double> result(3);
    result[0] = lhs.data[1] * rhs.data[2] - lhs.data[2] * rhs.data[1];
    result[1] = lhs.data[2] * rhs.data[0] - lhs.data[0] * rhs.data[2];
    result[2] = lhs.data[0] * rhs.data[1] - lhs.data[1] * rhs.data[0];

    return Vector(result);
}

double Vector::norm(const Vector& v) {
    return std::sqrt(Vector::dot(v, v));
}

Vector Vector::normalize(const Vector& v) {
    return v / Vector::norm(v);
}

double Vector::angle(const Vector& lhs, const Vector& rhs) {
    if (lhs.getSize() != rhs.getSize()) {
        throw std::invalid_argument("Cannot calculate angle between vectors of different sizes.");
    }

    return std::acos(Vector::dot(lhs, rhs) / (std::pow(Vector::norm(lhs), 2)));
}


Vector LERP(const Vector& v1, const Vector& v2, double t)
{
    return v1 + (v2 - v1) * t;
}

Vector SLERP(const Vector& v1, const Vector& v2, double t)
{
    double angle = Vector::angle(v1, v2);
    double s_angle = std::sin(angle);
    double s1 = std::sin((1 - t) * angle) / s_angle;
    double s2 = std::sin(t * angle) / s_angle;
    return s1 * v1 + s2 * v2;
}

std::ostream &linalg::operator<<(std::ostream &os, const Vector& v) {
    os << "[";
    for (size_t i = 0; i < v.getSize(); ++i) {
        os << v.get(i);
        if (i < v.getSize() - 1) {
            os << ", ";
        }
    }
    os << "]";

    return os;
}