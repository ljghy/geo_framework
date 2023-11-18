#include <geo/mesh/SubMesh.h>

void getSubMesh(const Mesh &mesh, const std::vector<size_t> &selectedFaces,
                Mesh &subMesh, std::vector<int> *vertexIndicesMapping,
                std::vector<int> *faceIndicesMapping)
{
    subMesh.flags = 0;

    subMesh.vertices.clear();
    subMesh.faces.clear();
    subMesh.faces.reserve(selectedFaces.size());
    subMesh.halfEdges.clear();

    std::vector<int> vertexIndicesMapping_(mesh.nV(), -1);
    int subNV = 0;

    for (size_t i : selectedFaces)
    {
        const auto &f = mesh.faces[i];
        for (int j = 0; j < 3; ++j)
        {
            if (vertexIndicesMapping_[f.indices(j)] == -1)
            {
                vertexIndicesMapping_[f.indices(j)] = subNV++;
                subMesh.vertices.emplace_back(
                    mesh.vertices[f.indices(j)].position);
            }
        }
    }

    if (faceIndicesMapping != nullptr)
        faceIndicesMapping->resize(mesh.nF(), -1);

    for (size_t i : selectedFaces)
    {
        const auto &f = mesh.faces[i];
        Eigen::Vector3i indices;
        for (int j = 0; j < 3; ++j)
            indices(j) = vertexIndicesMapping_[f.indices(j)];
        if (faceIndicesMapping != nullptr)
            (*faceIndicesMapping)[i] = static_cast<int>(subMesh.faces.size());
        subMesh.faces.emplace_back(indices);
    }
    if (vertexIndicesMapping != nullptr)
    {
        vertexIndicesMapping->swap(vertexIndicesMapping_);
    }
}