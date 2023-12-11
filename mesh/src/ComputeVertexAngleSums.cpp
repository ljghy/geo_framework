#include <geo/mesh/Mesh.h>

void Mesh::computeVertexAngleSums()
{
    require(FaceAngles);
    for (auto &vertex : vertices)
        vertex.angleSum = 0.0;

    for (const auto &face : faces)
        for (int j = 0; j < 3; ++j)
            vertices[face.indices(j)].angleSum += face.angles(j);

    flags |= VertexAngleSums;
}
