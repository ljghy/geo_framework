#ifndef GEO_DIHEDRAL_ANGLES_H_
#define GEO_DIHEDRAL_ANGLES_H_

#include <numbers>

#include <geo/mesh/Mesh.h>

std::vector<HalfEdge *>
dihedralAnglesThreshold(Mesh &mesh,
                        double threshold = std::numbers::pi / 3.0 * 2.0);

#endif
