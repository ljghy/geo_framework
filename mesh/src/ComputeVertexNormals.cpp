#include <geo/mesh/Mesh.h>

void Mesh::computeVertexNormals()
{
    require(FaceNormals | FaceAngles);

    for (auto &vertex : vertices)
        vertex.normal = Eigen::Vector3d::Zero();

    for (const auto &face : faces)
        for (int j = 0; j < 3; ++j)
            vertices[face.indices(j)].normal += face.angles(j) * face.normal;

    for (auto &vertex : vertices)
        vertex.normal.normalize();

    flags |= VertexNormals;
}
