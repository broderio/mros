#include "kineval/joint.hpp"

namespace kineval
{
    Joint::Joint()
    {
    }

    Joint::Joint(const linalg::Vector &axis, const linalg::Vector &origin, const JointType &type)
        : axis(axis), origin(origin), type(type)
    {
    }

    Joint::Joint(const Joint &j)
    {
        axis = j.axis;
        origin = j.origin;
        type = j.type;
    }

    Joint &Joint::operator=(const Joint &j)
    {
        if (this == &j)
        {
            return *this;
        }

        axis = j.axis;
        origin = j.origin;
        type = j.type;

        return *this;
    }

    Transform Joint::getTransform() const
    {
        linalg::Vector t(3);
        t.at(0) = origin.get(0);
        t.at(1) = origin.get(1);
        t.at(2) = origin.get(2);

        linalg::Vector rpy(3);
        rpy.at(0) = origin.get(3);
        rpy.at(1) = origin.get(4);
        rpy.at(2) = origin.get(5);
        linalg::Rotation R = linalg::Rotation::fromRPY(rpy);

        switch (type)
        {
        case REVOLUTE:
            R = linalg::Rotation::multiply(linalg::Rotation::fromAxisAngle(axis, q), R);
            break;
        case PRISMATIC:
            t += q * axis;
            break;
        }

        return Transform(t, R);
    }

    linalg::Vector Joint::getAxis() const
    {
        return axis;
    }

    linalg::Vector Joint::getOrigin() const
    {
        return origin;
    }

    JointType Joint::getType() const
    {
        return type;
    }

}