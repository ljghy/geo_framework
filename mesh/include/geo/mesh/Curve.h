#ifndef GEO_CURVE_H_
#define GEO_CURVE_H_

#include <vector>

#include <Eigen/Core>

struct Curve
{
    bool isLoop;
    std::vector<Eigen::Vector3d> vertices;
};

#endif
