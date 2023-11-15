#include <geo/mesh/Mesh.h>

std::vector<std::vector<int>> Mesh::getVertexOneRing()
{
    require(HalfEdgeStructure | VertexIndices);
    std::vector<std::vector<int>> oneRing(nV());
    for (const auto &vert : vertices)
    {
        int oneRingSize = 0;
        vert.forEachHalfEdge([&oneRingSize](const HalfEdge *)
                             { ++oneRingSize; });
        oneRing[vert.index].reserve(oneRingSize);
    }
    for (const auto &vert : vertices)
    {
        vert.forEachHalfEdge(
            [&oneRing, &vert](const HalfEdge *he)
            { oneRing[vert.index].push_back(he->twin->origin->index); });
    }
    return oneRing;
}

std::vector<std::vector<int>> Mesh::getVertexOneRingWithCenter()
{
    require(HalfEdgeStructure | VertexIndices);
    std::vector<std::vector<int>> oneRing(nV());
    for (const auto &vert : vertices)
    {
        int oneRingSize = 1;
        vert.forEachHalfEdge([&oneRingSize](const HalfEdge *)
                             { ++oneRingSize; });
        oneRing[vert.index].reserve(oneRingSize);
    }
    for (const auto &vert : vertices)
    {
        oneRing[vert.index].push_back(vert.index);
        vert.forEachHalfEdge(
            [&oneRing, &vert](const HalfEdge *he)
            { oneRing[vert.index].push_back(he->twin->origin->index); });
    }
    return oneRing;
}
