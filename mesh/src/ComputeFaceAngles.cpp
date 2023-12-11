#include <cmath>

#include <Eigen/Dense>

#include <geo/mesh/Mesh.h>

void Mesh::computeFaceAngles()
{
    for (auto &face : faces)
    {
        const auto &I = face.indices;
        Eigen::Vector3d e[3]{vertices[I(1)].position - vertices[I(0)].position,
                             vertices[I(2)].position - vertices[I(1)].position,
                             vertices[I(0)].position - vertices[I(2)].position};
        for (int j = 0; j < 3; ++j)
        {
            int k = (j + 2) % 3;
            double s = e[j].cross(e[k]).norm();
            double c = -e[j].dot(e[k]);
            face.angles(j) = std::atan2(s, c);
        }
    }

    flags |= FaceAngles;
}

void Mesh::computeIntrinsicFaceAngles(const Eigen::MatrixX3d &l)
{
    for (auto &face : faces)
    {
        auto f = getFaceIndex(&face);
        double s[3]{l(f, 0) * l(f, 0), l(f, 1) * l(f, 1), l(f, 2) * l(f, 2)};
        for (int i = 0; i < 3; ++i)
        {
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;
            face.angles(i) = std::acos(std::clamp(
                (s[j] + s[k] - s[i]) / (2.0 * l(f, j) * l(f, k)), -1.0, 1.0));
        }
    }

    flags |= FaceAngles;
}
