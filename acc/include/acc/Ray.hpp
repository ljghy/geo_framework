#ifndef ACC_RAY_HPP_
#define ACC_RAY_HPP_

#include <Eigen/Core>

struct Ray
{
    Eigen::Vector3d orig;
    Eigen::Vector3d dir;
};

#endif
