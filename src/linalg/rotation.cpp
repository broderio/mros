#include "linalg/rotation.hpp"

namespace linalg
{

    Rotation::Rotation()
    {
    }

    Rotation::Rotation(const Matrix &m) : Matrix(m)
    {
    }

    Rotation Rotation::identity()
    {
        return Matrix::identity(3);
    }

    Rotation Rotation::X(const float &angle)
    {
        Rotation r = Rotation::identity();
        r.data[1][1] = std::cos(angle);
        r.data[1][2] = -std::sin(angle);
        r.data[2][1] = std::sin(angle);
        r.data[2][2] = std::cos(angle);
        return r;
    }

    Rotation Rotation::Y(const float &angle)
    {
        Rotation r = Rotation::identity();
        r.data[0][0] = std::cos(angle);
        r.data[0][2] = std::sin(angle);
        r.data[2][0] = -std::sin(angle);
        r.data[2][2] = std::cos(angle);
        return r;
    }

    Rotation Rotation::Z(const float &angle)
    {
        Rotation r = Rotation::identity();
        r.data[0][0] = std::cos(angle);
        r.data[0][1] = -std::sin(angle);
        r.data[1][0] = std::sin(angle);
        r.data[1][1] = std::cos(angle);
        return r;
    }

    Rotation Rotation::fromQuaternion(const Quaternion &q)
    {
        return Quaternion::toRotationMatrix(q);
    }

    Rotation Rotation::fromRPY(const Vector &rpy)
    {
        if (rpy.getSize() != 3)
        {
            throw std::invalid_argument("RPY vector must have 3 elements");
        }

        Rotation RX = Rotation::X(rpy.get(0));
        Rotation RY = Rotation::Y(rpy.get(1));
        Rotation RZ = Rotation::Z(rpy.get(2));

        return Rotation::multiply(RZ, Rotation::multiply(RY, RX));
    }

    Rotation Rotation::fromAxisAngle(const Vector &axis, const float &angle)
    {
        if (axis.getSize() != 3)
        {
            throw std::invalid_argument("Axis vector must have 3 elements");
        }

        Vector anorm = Vector::normalize(axis);
        float kx = anorm.get(0);
        float ky = anorm.get(1);
        float kz = anorm.get(2);
        float c = std::cos(angle);
        float s = std::sin(angle);
        float v = 1 - c;

        std::vector<std::vector<float>> m = {
            {kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s},
            {kx * ky * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s},
            {kx * kz * v - ky * s, ky * kz * v + kx * s, kz * kz * v + c}};

        return Rotation(m);
    }

    Rotation Rotation::fromEulerZYZ(const Vector &euler)
    {
        if (euler.getSize() != 3)
        {
            throw std::invalid_argument("Euler vector must have 3 elements");
        }

        float alpha = euler.get(0);
        float beta = euler.get(1);
        float gamma = euler.get(2);

        Matrix R1 = Rotation::X(alpha);
        Matrix R2 = Rotation::Y(beta);
        Matrix R3 = Rotation::Z(gamma);

        return Rotation::multiply(R3, Rotation::multiply(R2, R1));
    }

    Rotation Rotation::fromEulerZYX(const Vector &euler)
    {
        if (euler.getSize() != 3)
        {
            throw std::invalid_argument("Euler vector must have 3 elements");
        }
        
        float alpha = euler.get(0);
        float beta = euler.get(1);
        float gamma = euler.get(2);

        Rotation R1 = Rotation::X(alpha);
        Rotation R2 = Rotation::Y(beta);
        Rotation R3 = Rotation::Z(gamma);

        return Rotation::multiply(R3, Rotation::multiply(R2, R1));
    }

    Rotation Rotation::multiply(const Rotation &r1, const Rotation &r2)
    {
        std::vector<std::vector<float>> m = {{r1.data[0][0] * r2.data[0][0] + r1.data[0][1] * r2.data[1][0] + r1.data[0][2] * r2.data[2][0],
                                              r1.data[0][0] * r2.data[0][1] + r1.data[0][1] * r2.data[1][1] + r1.data[0][2] * r2.data[2][1],
                                              r1.data[0][0] * r2.data[0][2] + r1.data[0][1] * r2.data[1][2] + r1.data[0][2] * r2.data[2][2]},
                                             {r1.data[1][0] * r2.data[0][0] + r1.data[1][1] * r2.data[1][0] + r1.data[1][2] * r2.data[2][0],
                                              r1.data[1][0] * r2.data[0][1] + r1.data[1][1] * r2.data[1][1] + r1.data[1][2] * r2.data[2][1],
                                              r1.data[1][0] * r2.data[0][2] + r1.data[1][1] * r2.data[1][2] + r1.data[1][2] * r2.data[2][2]},
                                             {r1.data[2][0] * r2.data[0][0] + r1.data[2][1] * r2.data[1][0] + r1.data[2][2] * r2.data[2][0],
                                              r1.data[2][0] * r2.data[0][1] + r1.data[2][1] * r2.data[1][1] + r1.data[2][2] * r2.data[2][1],
                                              r1.data[2][0] * r2.data[0][2] + r1.data[2][1] * r2.data[1][2] + r1.data[2][2] * r2.data[2][2]}};
        return Rotation(m);
    }

    Rotation Rotation::multiply(const Rotation &r, const linalg::Matrix &m)
    {
        if (m.getRows() != 3 || m.getCols() != 3)
        {
            throw std::invalid_argument("Matrix must be 3x3");
        }

        std::vector<std::vector<float>> mNew = {{r.data[0][0] * m.get(0, 0) + r.data[0][1] * m.get(1, 0) + r.data[0][2] * m.get(2, 0),
                                              r.data[0][0] * m.get(0, 1) + r.data[0][1] * m.get(1, 1) + r.data[0][2] * m.get(2, 1),
                                              r.data[0][0] * m.get(0, 2) + r.data[0][1] * m.get(1, 2) + r.data[0][2] * m.get(2, 2)},
                                             {r.data[1][0] * m.get(0, 0) + r.data[1][1] * m.get(1, 0) + r.data[1][2] * m.get(2, 0),
                                              r.data[1][0] * m.get(0, 1) + r.data[1][1] * m.get(1, 1) + r.data[1][2] * m.get(2, 1),
                                              r.data[1][0] * m.get(0, 2) + r.data[1][1] * m.get(1, 2) + r.data[1][2] * m.get(2, 2)},
                                             {r.data[2][0] * m.get(0, 0) + r.data[2][1] * m.get(1, 0) + r.data[2][2] * m.get(2, 0),
                                              r.data[2][0] * m.get(0, 1) + r.data[2][1] * m.get(1, 1) + r.data[2][2] * m.get(2, 1),
                                              r.data[2][0] * m.get(0, 2) + r.data[2][1] * m.get(1, 2) + r.data[2][2] * m.get(2, 2)}};
        return Rotation(mNew);
    }

    Vector Rotation::multiply(const Rotation &r, const linalg::Vector &v)
    {
        if (v.getSize() != 3)
        {
            throw std::invalid_argument("Vector must have 3 elements");
        }

        std::vector<float> vNew = {r.data[0][0] * v.get(0) + r.data[0][1] * v.get(1) + r.data[0][2] * v.get(2),
                                r.data[1][0] * v.get(0) + r.data[1][1] * v.get(1) + r.data[1][2] * v.get(2),
                                r.data[2][0] * v.get(0) + r.data[2][1] * v.get(1) + r.data[2][2] * v.get(2)};
        return Vector(vNew);
    }

    Rotation Rotation::inverse(const Rotation &r)
    {
        return Matrix::transpose(r);
    }

    Quaternion Rotation::getQuaternion() const
    {
        /**
         * This function was adapted from https://docs.ros.org/en/indigo/api/orocos_kdl/html/frames_8cpp_source.html#l00205
            double trace = (*this)(0,0) + (*this)(1,1) + (*this)(2,2);
            double epsilon=1E-12;
            if( trace > epsilon ){
                double s = 0.5 / sqrt(trace + 1.0);
                w = 0.25 / s;
                x = ( (*this)(2,1) - (*this)(1,2) ) * s;
                y = ( (*this)(0,2) - (*this)(2,0) ) * s;
                z = ( (*this)(1,0) - (*this)(0,1) ) * s;
            }else{
                if ( (*this)(0,0) > (*this)(1,1) && (*this)(0,0) > (*this)(2,2) ){
                    double s = 2.0 * sqrt( 1.0 + (*this)(0,0) - (*this)(1,1) - (*this)(2,2));
                    w = ((*this)(2,1) - (*this)(1,2) ) / s;
                    x = 0.25 * s;
                    y = ((*this)(0,1) + (*this)(1,0) ) / s;
                    z = ((*this)(0,2) + (*this)(2,0) ) / s;
                } else if ((*this)(1,1) > (*this)(2,2)) {
                    double s = 2.0 * sqrt( 1.0 + (*this)(1,1) - (*this)(0,0) - (*this)(2,2));
                    w = ((*this)(0,2) - (*this)(2,0) ) / s;
                    x = ((*this)(0,1) + (*this)(1,0) ) / s;
                    y = 0.25 * s;
                    z = ((*this)(1,2) + (*this)(2,1) ) / s;
                }else {
                    double s = 2.0 * sqrt( 1.0 + (*this)(2,2) - (*this)(0,0) - (*this)(1,1) );
                    w = ((*this)(1,0) - (*this)(0,1) ) / s;
                    x = ((*this)(0,2) + (*this)(2,0) ) / s;
                    y = ((*this)(1,2) + (*this)(2,1) ) / s;
                    z = 0.25 * s;
                }
            }
        */

        double w, x, y, z;
        double trace = get(0, 0) + get(1, 1) + get(2, 2);
        double epsilon = 1E-12;
        if (trace > epsilon)
        {
            double s = 0.5 / std::sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (get(2, 1) - get(1, 2)) * s;
            y = (get(0, 2) - get(2, 0)) * s;
            z = (get(1, 0) - get(0, 1)) * s;
        }
        else
        {
            if (get(0, 0) > get(1, 1) && get(0, 0) > get(2, 2))
            {
                double s = 2.0 * std::sqrt(1.0 + get(0, 0) - get(1, 1) - get(2, 2));
                w = (get(2, 1) - get(1, 2)) / s;
                x = 0.25 * s;
                y = (get(0, 1) + get(1, 0)) / s;
                z = (get(0, 2) + get(2, 0)) / s;
            }
            else if (get(1, 1) > get(2, 2))
            {
                double s = 2.0 * std::sqrt(1.0 + get(1, 1) - get(0, 0) - get(2, 2));
                w = (get(0, 2) - get(2, 0)) / s;
                x = (get(0, 1) + get(1, 0)) / s;
                y = 0.25 * s;
                z = (get(1, 2) + get(2, 1)) / s;
            }
            else
            {
                double s = 2.0 * std::sqrt(1.0 + get(2, 2) - get(0, 0) - get(1, 1));
                w = (get(1, 0) - get(0, 1)) / s;
                x = (get(0, 2) + get(2, 0)) / s;
                y = (get(1, 2) + get(2, 1)) / s;
                z = 0.25 * s;
            }
        }

        return Quaternion(x, y, z, w);
    }

    Vector Rotation::getRPY() const
    {
        Vector rpy(3);
        rpy.at(0) = std::atan2(get(2, 1), get(2, 2));
        rpy.at(1) = std::atan2(-get(2, 0), std::sqrt(get(2, 1) * get(2, 1) + get(2, 2) * get(2, 2)));
        rpy.at(2) = std::atan2(get(1, 0), get(0, 0));

        return rpy;
    }

    void Rotation::getAxisAngle(Vector &axis, float &angle) const
    {
        axis = Vector(3);
        float f = (get(0, 0) + get(1, 1) + get(2, 2) - 1) / 2;
        angle = acos(std::max(-1.0f, std::min(1.0f, f)));

        float s = 2 * std::sin(angle);
        axis.at(0) = (get(2, 1) - get(1, 2)) / s;
        axis.at(1) = (get(0, 2) - get(2, 0)) / s;
        axis.at(2) = (get(1, 0) - get(0, 1)) / s;
    }

    Vector Rotation::getEulerZYZ() const
    {
        /**
         * This function was adapted from https://docs.ros.org/en/indigo/api/orocos_kdl/html/frames_8cpp_source.html#l00276
        double epsilon = 1E-12;
        if (fabs(data[8]) > 1-epsilon  ) {
            gamma=0.0;
            if (data[8]>0) {
                beta = 0.0;
                alpha= atan2(data[3],data[0]);
            } else {
                beta = PI;
                alpha= atan2(-data[3],-data[0]);
            }
        } else {
            alpha=atan2(data[5], data[2]);
            beta=atan2(sqrt( sqr(data[6]) +sqr(data[7]) ),data[8]);
            gamma=atan2(data[7], -data[6]);
        }
        */

        Vector euler(3);
        double epsilon = 1E-12;

        if (std::fabs(get(2, 2)) > 1 - epsilon)
        {
            euler.at(2) = 0.0;
            if (get(2, 2) > 0)
            {
                euler.at(1) = 0.0;
                euler.at(0) = std::atan2(get(1, 0), get(0, 0));
            }
            else
            {
                euler.at(1) = M_PI;
                euler.at(0) = std::atan2(-get(1, 0), -get(0, 0));
            }
        }
        else
        {
            euler.at(0) = std::atan2(get(1, 2), get(0, 2));
            euler.at(1) = std::atan2(std::sqrt(std::pow(get(2, 0), 2) + std::pow(get(2, 1), 2)), get(2, 2));
            euler.at(2) = std::atan2(get(2, 1), -get(2, 0));
        }
    }

    Vector Rotation::getEulerZYX() const
    {
        return getRPY();
    }
}