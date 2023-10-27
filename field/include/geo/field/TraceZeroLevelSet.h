#ifndef GEO_TRACE_ZERO_LEVEL_SET_H_
#define GEO_TRACE_ZERO_LEVEL_SET_H_

#include <vector>

#include <Eigen/Core>

#include <geo/mesh/Mesh.h>
#include <geo/mesh/Curve.h>

std::vector<Curve> traceZeroLevelSet(Mesh &mesh, const Eigen::VectorXd &phi,
                                     double dupThreshold = 1e-5);

#endif
