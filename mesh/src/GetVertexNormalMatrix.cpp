#include <geo/mesh/Mesh.h>

Eigen::Matrix3Xd Mesh::getVertexNormalMatrix()
{
    require(VertexNormals);
    Eigen::Matrix3Xd N(3, nV());
    for (size_t i = 0; i < nV(); i++)
        N.col(i) = vertices[i].normal;
    return N;
}
