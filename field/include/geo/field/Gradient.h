#ifndef GEO_GRADIENT_H_
#define GEO_GRADIENT_H_

#include <geo/mesh/Mesh.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>

void computeGradient(Mesh &mesh, const Eigen::VectorXd &phi,
                     Eigen::Matrix3Xd &g);

#endif
