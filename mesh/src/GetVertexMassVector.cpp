#include <geo/mesh/Mesh.h>

Eigen::VectorXd Mesh::getVertexMassVector()
{
    require(VertexAreas);
    Eigen::VectorXd m(nV());
    for (size_t i = 0; i < nV(); i++)
        m(i) = vertices[i].area;
    return m;
}