#include <cmath>
#include <numbers>

#include <Eigen/Dense>

#include <geo/mesh/Mesh.h>

static double halfEdgeAngle(const HalfEdge *he)
{
    const Vertex *v = he->origin;
    if (he == v->he)
        return 0.0;
    Eigen::Vector3d t = v->he->vec();
    Eigen::Vector3d b = v->normal.cross(t).normalized();
    t = b.cross(v->normal).normalized();

    Eigen::Vector3d u = he->vec();
    return std::atan2(u.dot(b), u.dot(t));
}

void Mesh::computeConnectionAngles()
{
    require(HalfEdgeStructure | VertexNormals);

    for (auto &he : halfEdges)
        he->connectionAngle = halfEdgeAngle(he->twin) + std::numbers::pi -
                              halfEdgeAngle(he.get());

    flags |= ConnectionAngles;
}
