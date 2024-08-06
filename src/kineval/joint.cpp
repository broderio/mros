#include "kineval/joint.hpp"

namespace kineval
{
    Joint::Joint()
    {
    }

    Joint::Joint(const linalg::Vector &axis, const linalg::Vector &origin, const JointType &type)
        : axis(axis), origin(origin), type(type), limited(false)
    {
    }

    Joint::Joint(const Joint &j)
    {
        axis = j.axis;
        origin = j.origin;
        type = j.type;
        limited = j.limited;
        upperBound = j.upperBound;
        lowerBound = j.lowerBound;
        q = j.q;
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
        limited = j.limited;
        upperBound = j.upperBound;
        lowerBound = j.lowerBound;
        q = j.q;

        return *this;
    }

    Transform Joint::getTransform() const
    {
        linalg::Vector t(3);
        t.at(0) = origin.get(0);
        t.at(1) = origin.get(1);
        t.at(2) = origin.get(2);

        linalg::Quaternion r = linalg::Quaternion::fromRPY(origin.get(3), origin.get(4), origin.get(5));

        if (type == REVOLUTE || type == CONTINUOUS)
        {
            r = linalg::Quaternion::multiply(r, linalg::Quaternion(axis, q));
        }
        else if (type == PRISMATIC)
        {
            t += q * axis;
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

    void Joint::setLimits(double lowerBound, double upperBound)
    {
        this->limited = true;
        this->lowerBound = lowerBound;
        this->upperBound = upperBound;
    }

    bool Joint::getLimits(double &lowerBound, double &upperBound) const
    {
        if (this->limited)
        {
            lowerBound = this->lowerBound;
            upperBound = this->upperBound;
        }
        return this->limited;
    }

    bool Joint::hasLimits() const
    {
        return this->limited;
    }

    bool Joint::inLimits(double q) const
    {
        if (!limited)
        {
            return true;
        }
        return (q < upperBound) && (q > lowerBound);
    }

    void Joint::removeLimits()
    {
        this->limited = false;
    }

    void Joint::setState(double q)
    {
        if (hasLimits())
        {
            this->q = std::max(lowerBound, std::min(q, upperBound));
        }
        else
        {
            this->q = q;
        }
    }

    double Joint::getState() const
    {
        return q;
    }
}