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
                             { oneRingMap[i][j++] = he->twin->origin->index; });
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
        oneRingMap[i][0] = i;
        vert.forEachHalfEdge([&oneRingMap, i, &j](const HalfEdge *he)
                             { oneRingMap[i][j++] = he->twin->origin->index; });
    }
    return oneRingMap;
}
