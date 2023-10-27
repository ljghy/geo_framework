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
