#include <algorithm>

#include <geo/mesh/Components.h>
#include <geo/mesh/SubMesh.h>

std::vector<std::shared_ptr<Mesh>> getConnectedComponents(Mesh &mesh)
{
    mesh.require(Mesh::HalfEdgeStructure | Mesh::FaceIndices);

    std::vector<size_t> selectedFaces;
    selectedFaces.reserve(mesh.nF());

    std::vector<std::shared_ptr<Mesh>> components;

    std::vector<bool> visited(mesh.nF(), false);
    while (true)
    {
        selectedFaces.clear();

        auto it = std::find(visited.begin(), visited.end(), false);
        if (it == visited.end())
            break;

        size_t i = std::distance(visited.begin(), it);
        selectedFaces.push_back(i);
        visited[i] = true;

        size_t j = 0;
        while (j != selectedFaces.size())
        {
            const auto &f = mesh.faces[selectedFaces[j]];
            const auto *he = f.he;
            for (int k = 0; k < 3; ++k, he = he->next)
            {
                if (he->twin->face == nullptr || visited[he->twin->face->index])
                    continue;
                selectedFaces.push_back(he->twin->face->index);
                visited[he->twin->face->index] = true;
            }
            ++j;
        }
        getSubMesh(mesh, selectedFaces, *components.emplace_back(new Mesh()));
    }

    return components;
}
