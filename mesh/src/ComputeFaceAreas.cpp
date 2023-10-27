#include <Eigen/Dense>

#include <geo/mesh/Mesh.h>

void Mesh::computeFaceAreas()
{
    for (auto &face : faces)
    {
        const auto &I = face.indices;
        Eigen::Vector3d e1 = vertices[I(1)].position - vertices[I(0)].position;
        Eigen::Vector3d e2 = vertices[I(2)].position - vertices[I(0)].position;
        face.area = 0.5 * e2.cross(e1).norm();
    }

    flags |= FaceAreas;
}
