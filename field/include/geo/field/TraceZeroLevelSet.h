#ifndef GEO_TRACE_ZERO_LEVEL_SET_H_
#define GEO_TRACE_ZERO_LEVEL_SET_H_

#include <vector>
#include <memory>

#include <Eigen/Core>

#include <geo/mesh/Mesh.h>
#include <geo/mesh/Curve.h>

std::vector<std::shared_ptr<Curve>>
traceZeroLevelSet(Mesh &mesh, const Eigen::VectorXd &phi,
                  double dupThreshold = 1e-5);

#endif
