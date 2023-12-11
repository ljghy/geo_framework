#include <geo/mesh/Mesh.h>

Eigen::MatrixX3d Mesh::getEdgeLengthMatrix() const
{
    Eigen::MatrixX3d l(nF(), 3);
    for (size_t i = 0; i < nF(); ++i)
    {
        const auto &I = faces[i].indices;
        for (int j = 0; j < 3; ++j)
        {
            auto p = I((j + 1) % 3);
            auto q = I((j + 2) % 3);
            l(i, j) = (vertices[p].position - vertices[q].position).norm();
        }
    }
    return l;
}
