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

        Transform &operator=(const Transform &f);

        void applyTranslation(const linalg::Vector &translation);
        
        void applyRotation(const linalg::Quaternion &rotation); // Extrinsic rotation

        void applyTransform(const Transform &transform);

        linalg::Vector getTranslation() const;

        linalg::Quaternion getRotation() const;

        linalg::Matrix getTransform() const;

        void setTranslation(const linalg::Vector &translation);

        void setRotation(const linalg::Quaternion &rotation);

    private:

        linalg::Vector translation;
        linalg::Quaternion rotation;

    };

} // namespace kineval