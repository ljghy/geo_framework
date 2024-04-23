#ifndef GEO_INDEX_BASED_HALF_EDGE_H_
#define GEO_INDEX_BASED_HALF_EDGE_H_

#include <vector>
#include <memory>

#include <geo/mesh/Mesh.h>

struct IndexBasedHalfEdge
{
    int origin = -1;
    int twin = -1;
    int prev = -1;
    int next = -1;
    int face = -1;
};

struct IndexBasedHalfEdgeStructure
{
    std::vector<std::unique_ptr<IndexBasedHalfEdge>> halfEdges;
    std::vector<int> vertHalfEdge;
    std::vector<int> faceHalfEdge;

    void construct(Mesh &mesh);
    void toPointerBased(Mesh &mesh);
};

#endif
