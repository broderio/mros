#pragma once

#include <iostream>

#include "kineval/transform.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/rotation.hpp"

namespace kineval
{

    class Link
    {
    public:
        Link();
        Link(const linalg::Vector &origin);
        Link(const Link &other);
        Link &operator=(const Link &other);

        Transform getTransform() const;
        linalg::Vector getOrigin() const;

    private:
        linalg::Vector origin;
    };
} // namespace kineval