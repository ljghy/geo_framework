#ifndef GEO_SEG2SDF_H_
#define GEO_SEG2SDF_H_

#include <geo/mesh/Mesh.h>

void seg2SDF(Mesh &mesh, const std::vector<int> &segs, int requiredSeg,
             Eigen::VectorXd &phi);

#endif
