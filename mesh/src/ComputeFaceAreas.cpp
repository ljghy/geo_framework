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

void Mesh::computeIntrinsicFaceAreas(const Eigen::MatrixX3d &l)
{
    for (auto &face : faces)
    {
        auto f = getFaceIndex(&face);
        double p = 0.5 * (l(f, 0) + l(f, 1) + l(f, 2));
        face.area =
            std::sqrt(p * (p - l(f, 0)) * (p - l(f, 1)) * (p - l(f, 2)));
    }

    flags |= FaceAreas;
}
