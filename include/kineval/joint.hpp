#pragma once

#include <iostream>

#include "kineval/transform.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/rotation.hpp"

namespace kineval
{
    enum JointType
    {
        REVOLUTE,
        PRISMATIC
    };

    class Joint
    {
    public:
        Joint();
        Joint(const linalg::Vector &axis, const linalg::Vector &origin, const JointType &type);
        Joint(const Joint &j);
        Joint &operator=(const Joint &j);

        Transform getTransform() const;
        linalg::Vector getAxis() const;
        linalg::Vector getOrigin() const;
        JointType getType() const;

        float q;

    private:
        linalg::Vector axis;
        linalg::Vector origin;
        JointType type;
    };
} // namespace kineval