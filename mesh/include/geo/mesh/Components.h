#ifndef GEO_COMPONENTS_H_
#define GEO_COMPONENTS_H_

#include <memory>

#include <geo/mesh/Mesh.h>

std::vector<std::shared_ptr<Mesh>> getConnectedComponents(Mesh &mesh);

#endif
