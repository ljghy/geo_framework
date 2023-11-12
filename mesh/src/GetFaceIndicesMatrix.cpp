#include <geo/mesh/Mesh.h>

Eigen::Matrix3Xi Mesh::getFaceIndicesMatrix() const
{
    Eigen::Matrix3Xi F(3, nF());
    for (size_t i = 0; i < nF(); i++)
        F.col(i) = faces[i].indices;
    return F;
}

Eigen::MatrixX3i Mesh::getFaceIndicesMatrixTransposed() const
{
    Eigen::MatrixX3i F(nF(), 3);
    for (size_t i = 0; i < nF(); i++)
        F.row(i) = faces[i].indices;
    return F;
}
