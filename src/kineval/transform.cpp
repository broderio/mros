#include "kineval/transform.hpp"

namespace kineval
{
    Transform::Transform()
    : translation(3, 0), rotation(linalg::Quaternion::identity())
    {
    }

    Transform::Transform(const linalg::Vector &translation, const linalg::Quaternion &rotation)
    {
        if (translation.getSize() != 3)
        {
            throw std::invalid_argument("Translation vector must have size 3");
        }

        this->translation = translation;
        this->rotation = rotation;
    }

    Transform::Transform(const Transform &other)
    {
        translation = other.translation;
        rotation = other.rotation;
    }

    Transform Transform::identity()
    {
        return Transform(linalg::Vector(3, 0), linalg::Quaternion::identity());
    }

    Transform &Transform::operator=(const Transform &other)
    {
        if (this == &other)
        {
            return *this;
        }

        translation = other.translation;
        rotation = other.rotation;

        return *this;
    }

    Transform Transform::applyRotation(const linalg::Quaternion &rotation) const
    {
        linalg::Vector t = linalg::Quaternion::rotate(this->translation, rotation);
        linalg::Quaternion r = linalg::Quaternion::multiply(this->rotation, rotation);
        return Transform(t, r);
    }

    Transform Transform::applyTranslation(const linalg::Vector &translation) const
    {
        if (translation.getSize() != 3)
        {
            throw std::invalid_argument("Translation vector must have size 3");
        }
        return Transform(this->translation + translation, this->rotation);
    }

    Transform Transform::applyTransform(const Transform &transform) const
    {
        linalg::Vector t = this->translation + linalg::Quaternion::rotate(transform.translation, this->rotation);
        linalg::Quaternion r = linalg::Quaternion::multiply(this->rotation, transform.rotation);
        return Transform(t, r);
    }

    linalg::Vector Transform::getTranslation() const
    {
        return translation;
    }

    linalg::Quaternion Transform::getRotation() const
    {
        return rotation;
    }

    linalg::Matrix Transform::getMatrix() const
    {
        linalg::Matrix T(4, 4);
        T.setSubmatrix(0, 0, linalg::Quaternion::toRotationMatrix(rotation));
        T.at(0, 3) = translation.get(0);
        T.at(1, 3) = translation.get(1);
        T.at(2, 3) = translation.get(2);
        T.set(3, 3, 1);
        return T;      
    }

    void Transform::setTranslation(const linalg::Vector &translation)
    {
        this->translation = translation;
    }

    void Transform::setRotation(const linalg::Quaternion &rotation)
    {
        this->rotation = rotation;
    }

} // namespace kineval