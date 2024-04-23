#include <unordered_set>
#include <cmath>

#include <geo/mesh/DihedralAngles.h>

std::vector<HalfEdge *> dihedralAnglesThreshold(Mesh &mesh, double threshold)
{
    mesh.require(Mesh::HalfEdgeStructure | Mesh::FaceNormals);
    std::vector<HalfEdge *> ret;

    double cosThreshold = -std::cos(threshold);

    std::unordered_set<HalfEdge *> visited;

    for (const auto &he : mesh.halfEdges)
    {
        if (visited.contains(he.get()))
            continue;
        auto *twin = he->twin;
        if (he->face == nullptr || twin->face == nullptr)
            continue;
        visited.insert(twin);
        if (he->face->normal.dot(twin->face->normal) < cosThreshold)
            ret.push_back(he.get());
    }
    return ret;
}
