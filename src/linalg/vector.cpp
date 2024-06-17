#include "linalg/vector.hpp"

Vector::Vector() : data() {}

Vector::Vector(size_t size) : data(size) {}

Vector::Vector(const Vector& v) : data(v.data) {}

Vector &Vector::operator=(const Vector& rhs) {
    if (this == &rhs) {
        return *this;
    }
    data = rhs.data;
    return *this;
}

Vector::Vector(const std::vector<float>& data) : data(data) {}

Vector::Vector(const Vector3& v) : data({v.x, v.y, v.z}) {}

std::vector<float> Vector::getData() const {
    return data;
}

float Vector::get(size_t idx) const {
    if (idx >= data.size()) {
        std::cerr << "Error: index out of bounds." << std::endl;
        return 0;
    }

    return data[idx];
}

float &Vector::at(size_t idx) {
    if (idx >= data.size()) {
        std::cerr << "Error: index out of bounds." << std::endl;
        return data[0];
    }

    return data[idx];
}

void Vector::set(size_t idx, float val) {
    if (idx >= data.size()) {
        std::cerr << "Error: index out of bounds." << std::endl;
        return;
    }

    data[idx] = val;
}

size_t Vector::getSize() const {
    return data.size();
}


Vector Vector::operator-() const {
    Vector v(*this);
    std::for_each(v.data.begin(), v.data.end(), [](float& f) { f = -f; });
    return v;
}


Vector Vector::operator+(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot add vectors of different sizes." << std::endl;
        return Vector();
    }

    Vector v(data.size());
    std::transform(data.begin(), data.end(), rhs.data.begin(), v.data.begin(), std::plus<float>());
    return v;
}

Vector Vector::operator+(const float& rhs) const {
    Vector v(data.size());
    std::transform(data.begin(), data.end(), v.data.begin(), [rhs](float f) { return f + rhs; });
    return v;
}


Vector Vector::operator-(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot subtract vectors of different sizes." << std::endl;
        return Vector();
    }

    Vector v(data.size());
    std::transform(data.begin(), data.end(), rhs.data.begin(), v.data.begin(), std::minus<float>());
    return v;
}

Vector Vector::operator-(const float& rhs) const {
    Vector v(data.size());
    std::transform(data.begin(), data.end(), v.data.begin(), [rhs](float f) { return f - rhs; });
    return v;
}


Vector Vector::operator*(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot multiply vectors of different sizes." << std::endl;
        return Vector();
    }

    Vector v(data.size());
    std::transform(data.begin(), data.end(), rhs.data.begin(), v.data.begin(), std::multiplies<float>());
    return v;
}

Vector Vector::operator*(const float& rhs) const {
    Vector v(data.size());
    std::transform(data.begin(), data.end(), v.data.begin(), [rhs](float f) { return f * rhs; });
    return v;
}


Vector Vector::operator/(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot divide vectors of different sizes." << std::endl;
        return Vector();
    }

    Vector v(data.size());
    std::transform(data.begin(), data.end(), rhs.data.begin(), v.data.begin(), std::divides<float>());
    return v;
}

Vector Vector::operator/(const float& rhs) const {
    Vector v(data.size());
    std::transform(data.begin(), data.end(), v.data.begin(), [rhs](float f) { return f / rhs; });
    return v;
}


Vector Vector::operator+=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot add vectors of different sizes." << std::endl;
        return *this;
    }

    std::transform(data.begin(), data.end(), rhs.data.begin(), data.begin(), std::plus<float>());
    return *this;
}

Vector Vector::operator+=(const float& rhs) {
    std::transform(data.begin(), data.end(), data.begin(), [rhs](float f) { return f + rhs; });
    return *this;
}


Vector Vector::operator-=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot subtract vectors of different sizes." << std::endl;
        return *this;
    }

    std::transform(data.begin(), data.end(), rhs.data.begin(), data.begin(), std::minus<float>());
    return *this;
}

Vector Vector::operator-=(const float& rhs) {
    std::transform(data.begin(), data.end(), data.begin(), [rhs](float f) { return f - rhs; });
    return *this;
}


Vector Vector::operator*=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot multiply vectors of different sizes." << std::endl;
        return *this;
    }

    std::transform(data.begin(), data.end(), rhs.data.begin(), data.begin(), std::multiplies<float>());
    return *this;
}

Vector Vector::operator*=(const float& rhs) {
    std::transform(data.begin(), data.end(), data.begin(), [rhs](float f) { return f * rhs; });
    return *this;
}


Vector Vector::operator/=(const Vector& rhs) {
    if (data.size() != rhs.data.size()) {
        std::cerr << "Error: cannot divide vectors of different sizes." << std::endl;
        return *this;
    }

    std::transform(data.begin(), data.end(), rhs.data.begin(), data.begin(), std::divides<float>());
    return *this;
}

Vector Vector::operator/=(const float& rhs) {
    std::transform(data.begin(), data.end(), data.begin(), [rhs](float f) { return f / rhs; });
    return *this;
}


bool Vector::operator==(const Vector& rhs) const {
    if (data.size() != rhs.data.size()) {
        return false;
    }

    return std::equal(data.begin(), data.end(), rhs.data.begin());
}

bool Vector::operator!=(const Vector& rhs) const {
    return !(*this == rhs);
}


Vector operator+(const float& lhs, const Vector& rhs) {
    return rhs + lhs;
}

Vector operator-(const float& lhs, const Vector& rhs) {
    return -rhs + lhs;
}

Vector operator*(const float& lhs, const Vector& rhs) {
    return rhs * lhs;
}


float Vector::dot(const Vector& lhs, const Vector& rhs) {
    if (lhs.getSize() != rhs.getSize()) {
        std::cerr << "Error: cannot take dot product of vectors of different sizes." << std::endl;
        return 0;
    }

    float result = 0;
    std::vector<float>::const_iterator r = rhs.data.begin(); 
    std::for_each(lhs.data.begin(), lhs.data.end(), [&result, &r](float f) { result += f * *r++; });

    return result;
}

Vector Vector::cross(const Vector& lhs, const Vector& rhs) {
    if (lhs.getSize() != 3 || rhs.getSize() != 3) {
        std::cerr << "Error: cannot take cross product of vectors that are not 3-dimensional." << std::endl;
        return Vector();
    }

    std::vector<float> result(3);
    result[0] = lhs.data[1] * rhs.data[2] - lhs.data[2] * rhs.data[1];
    result[1] = lhs.data[2] * rhs.data[0] - lhs.data[0] * rhs.data[2];
    result[2] = lhs.data[0] * rhs.data[1] - lhs.data[1] * rhs.data[0];

    return Vector(result);
}

float Vector::norm(const Vector& v) {
    return std::sqrt(Vector::dot(v, v));
}

Vector Vector::normalize(const Vector& v) {
    return v / Vector::norm(v);
}

float Vector::angle(const Vector& lhs, const Vector& rhs) {
    if (lhs.getSize() != rhs.getSize()) {
        std::cerr << "Error: cannot calculate angle between vectors of different sizes." << std::endl;
        return 0;
    }

    return std::acos(Vector::dot(lhs, rhs) / (std::pow(Vector::norm(lhs), 2)));
}

std::ostream &operator<<(std::ostream &os, const Vector& v) {
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