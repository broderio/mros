#include "kineval/transform.hpp"

namespace kineval
{
    Transform::Transform()
    {
    }

    Transform::Transform(const linalg::Vector &translation, const linalg::Quaternion &rotation)
    : translation(translation), rotation(rotation)
    {
        
    }

    Transform::Transform(const Transform &f)
    {
        translation = f.translation;
        rotation = f.rotation;    
    }

    Transform &Transform::operator=(const Transform &f)
    {
        if (this == &f)
        {
            return *this;
        }

        translation = f.translation;
        rotation = f.rotation;

        return *this;
    }

    void Transform::applyRotation(const linalg::Quaternion &rotation)
    {
        this->rotation = linalg::Quaternion::multiply(rotation, this->rotation);
    }

    void Transform::applyTranslation(const linalg::Vector &translation)
    {
        this->translation += translation;
    }

    void Transform::applyTransform(const Transform &transform)
    {
        this->rotation = linalg::Quaternion::multiply(this->rotation, transform.rotation);
        this->translation = linalg::Quaternion::rotate(this->translation, transform.rotation) + transform.translation;
    }

    linalg::Vector Transform::getTranslation() const
    {
        return translation;
    }

    linalg::Quaternion Transform::getRotation() const
    {
        return rotation;
    }

    linalg::Matrix Transform::getTransform() const
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