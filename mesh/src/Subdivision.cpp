#include <unordered_map>

#include <geo/mesh/PairHash.h>
#include <geo/mesh/Subdivision.h>

std::shared_ptr<Mesh>
midpointSubdivision(const Mesh &mesh,
                    std::vector<std::pair<int, int>> *newVertToEdge)
{
    auto ret = std::make_shared<Mesh>();

    ret->vertices.reserve(mesh.nV() * 4);
    ret->faces.resize(mesh.nF() * 4);
    if (newVertToEdge)
    {
        newVertToEdge->clear();
        newVertToEdge->reserve(mesh.nV());
    }

    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        ret->vertices.emplace_back(mesh.vertices[i].position);
    }
    std::unordered_map<std::pair<int, int>, int, pair_hash> edgeToVertex;
    for (size_t i = 0; i < mesh.nF(); ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            std::pair<int, int> p =
                std::minmax(mesh.F(i, j), mesh.F(i, (j + 1) % 3));
            auto [iter, success] = edgeToVertex.insert({p, ret->nV()});
            if (success)
            {
                ret->faces[i * 4].indices(j) = ret->nV();
                ret->vertices.emplace_back(
                    (mesh.V(p.first).position + mesh.V(p.second).position) *
                    0.5);
                if (newVertToEdge)
                {
                    newVertToEdge->emplace_back(p);
                }
            }
            else
            {
                ret->faces[i * 4].indices(j) = iter->second;
            }
        }
        for (int j = 0; j < 3; ++j)
        {
            int k = i * 4 + j + 1;
            ret->F(k, 0) = mesh.faces[i].indices(j);
            ret->F(k, 1) = ret->faces[i * 4].indices(j);
            ret->F(k, 2) = ret->faces[i * 4].indices((j + 2) % 3);
        }
    }

    return ret;
}
