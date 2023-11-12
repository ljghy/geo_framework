#include <geo/mesh/Mesh.h>

Eigen::Matrix3Xd Mesh::getVertexPositionMatrix() const
{
    Eigen::Matrix3Xd P(3, nV());
    for (size_t i = 0; i < nV(); i++)
        P.col(i) = vertices[i].position;
    return P;
}

Eigen::MatrixX3d Mesh::getVertexPositionMatrixTransposed() const
{
    Eigen::MatrixX3d P(nV(), 3);
    for (size_t i = 0; i < nV(); i++)
        P.row(i) = vertices[i].position;
    return P;
}
