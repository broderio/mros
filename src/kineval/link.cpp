#include "kineval/link.hpp"

namespace kineval
{
    Link::Link()
    {
    }

    Link::Link(const linalg::Vector &origin)
        : origin(origin)
    {
    }

    Link::Link(const Link &j)
    {
        origin = j.origin;
    }

    Link &Link::operator=(const Link &j)
    {
        if (this == &j)
        {
            return *this;
        }

        origin = j.origin;

        return *this;
    }

    Transform Link::getTransform() const
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

        return Transform(t, R);
    }

    linalg::Vector Link::getOrigin() const
    {
        return origin;
    }

}