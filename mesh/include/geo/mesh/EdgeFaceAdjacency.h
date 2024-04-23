#ifndef GEO_EDGE_FACE_ADJACENCY_H_
#define GEO_EDGE_FACE_ADJACENCY_H_

#include <unordered_map>

#include <geo/mesh/PairHash.h>
#include <geo/mesh/Mesh.h>

std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash>
getEdgeFaceAdjacency(Mesh &mesh);

#endif
