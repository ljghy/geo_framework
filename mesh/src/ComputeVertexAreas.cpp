#include <geo/mesh/Mesh.h>

void Mesh::computeVertexAreas()
{
    require(FaceAreas);
    for (auto &vertex : vertices)
        vertex.area = 0.0;

    for (const auto &face : faces)
        for (int j = 0; j < 3; ++j)
            vertices[face.indices(j)].area += face.area / 3.0;

    flags |= VertexAreas;
}
