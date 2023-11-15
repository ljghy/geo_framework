#include <geo/mesh/Mesh.h>

std::vector<std::map<int, int>> Mesh::getVertexOneRingMap()
{
    require(HalfEdgeStructure | VertexIndices);
    std::vector<std::map<int, int>> oneRingMap(nV());
    for (const auto &vert : vertices)
    {
        int i = vert.index;
        int j = 0;
        vert.forEachHalfEdge([&oneRingMap, i, &j](const HalfEdge *he)
                             { oneRingMap[i][he->twin->origin->index] = j++; });
    }
    return oneRingMap;
}

std::vector<std::map<int, int>> Mesh::getVertexOneRingMapWithCenter()
{
    require(HalfEdgeStructure | VertexIndices);
    std::vector<std::map<int, int>> oneRingMap(nV());
    for (const auto &vert : vertices)
    {
        int i = vert.index;
        int j = 1;
        oneRingMap[i][i] = 0;
        vert.forEachHalfEdge([&oneRingMap, i, &j](const HalfEdge *he)
                             { oneRingMap[i][he->twin->origin->index] = j++; });
    }
    return oneRingMap;
}
