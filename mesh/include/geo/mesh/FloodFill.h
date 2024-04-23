#ifndef GEO_FLOOD_FILL_H_
#define GEO_FLOOD_FILL_H_

#include <geo/mesh/Mesh.h>

void floodFillFace(Mesh &mesh, const std::vector<int> &barrier, int startFace,
                   std::vector<int> &filledFaces);

#endif
