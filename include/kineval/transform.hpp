#pragma once

#include <iostream>

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"
#include "linalg/rotation.hpp"

namespace kineval
{

    class Transform
    {
    public:
        Transform();

        Transform(const linalg::Vector &translation, const linalg::Quaternion &rotation);

        Transform(const Transform &f);

        static Transform identity();

        Transform &operator=(const Transform &f);

        Transform applyTranslation(const linalg::Vector &translation) const;
        
        Transform applyRotation(const linalg::Quaternion &rotation) const;

        Transform applyTransform(const Transform &transform) const;

        linalg::Vector getTranslation() const;

        linalg::Quaternion getRotation() const;

        linalg::Matrix getMatrix() const;

        void setTranslation(const linalg::Vector &translation);

        void setRotation(const linalg::Quaternion &rotation);

    private:

        linalg::Vector translation;
        linalg::Quaternion rotation;

    };

} // namespace kineval