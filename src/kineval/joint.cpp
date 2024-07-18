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

        linalg::Quaternion r = linalg::Quaternion::fromRPY(origin.get(3), origin.get(4), origin.get(5));

        switch (type)
        {
        case REVOLUTE:
            r = linalg::Quaternion::multiply(r, linalg::Quaternion(axis, q));
            break;
        case PRISMATIC:
            t += q * axis;
            break;
        }

        return Transform(t, r);
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