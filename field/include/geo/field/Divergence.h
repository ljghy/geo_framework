#ifndef GEO_DIVERGENCE_H_
#define GEO_DIVERGENCE_H_

#include <geo/mesh/Mesh.h>

void computeDivergence(Mesh &mesh, const Eigen::Matrix3Xd &X,
                       Eigen::VectorXd &div);

#endif
