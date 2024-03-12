#ifndef GEO_MESH_SUBDIVISION_H_
#define GEO_MESH_SUBDIVISION_H_

#include <memory>

#include <geo/mesh/Mesh.h>

std::shared_ptr<Mesh>
midpointSubdivision(const Mesh &mesh,
                    std::vector<std::pair<int, int>> *newVertToEdge = nullptr);

#endif
