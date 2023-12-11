#ifndef GEO_GRADIENT_H_
#define GEO_GRADIENT_H_

#include <geo/mesh/Mesh.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>

void computeGradient(Mesh &mesh, const Eigen::VectorXd &phi,
                     Eigen::Matrix3Xd &g);

std::vector<Eigen::Matrix3d> gradientOperator(Mesh &mesh);

std::vector<Eigen::VectorXcd>
intrinsicVertexGradientOperator(Mesh &mesh, const Eigen::MatrixX3d &l);

#endif
