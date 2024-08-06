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
        FIXED,
        CONTINUOUS,
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

        void setLimits(double lowerBound, double upperBound);
        bool getLimits(double &lowerBound, double &upperBound) const;
        bool hasLimits() const;
        bool inLimits(double q) const;
        void removeLimits();

        void setState(double q);
        double getState() const;

    private:
        linalg::Vector axis;
        linalg::Vector origin;
        JointType type;

        bool limited;
        double lowerBound;
        double upperBound;

        double q;
    };
} // namespace kineval