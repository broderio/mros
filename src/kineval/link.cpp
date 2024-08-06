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

    Link::Link(const Link &other)
    {
        origin = other.origin;
    }

    Link &Link::operator=(const Link &other)
    {
        if (this == &other)
        {
            return *this;
        }

        origin = other.origin;

        return *this;
    }

    Transform Link::getTransform() const
    {
        linalg::Vector t(3);
        t.at(0) = origin.get(0);
        t.at(1) = origin.get(1);
        t.at(2) = origin.get(2);

        linalg::Quaternion r = linalg::Quaternion::fromRPY(origin.get(3), origin.get(4), origin.get(5));

        return Transform(t, r);
    }

    linalg::Vector Link::getOrigin() const
    {
        return origin;
    }

}