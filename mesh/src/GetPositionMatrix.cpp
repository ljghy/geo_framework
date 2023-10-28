#include <geo/mesh/Mesh.h>

Eigen::Matrix3Xd Mesh::getPositionMatrix() const
{
    Eigen::Matrix3Xd P(3, nV());
    for (size_t i = 0; i < nV(); i++)
        P.col(i) = vertices[i].position;
    return P;
}
